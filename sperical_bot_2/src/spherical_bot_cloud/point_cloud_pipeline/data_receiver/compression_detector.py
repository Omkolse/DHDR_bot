#!/usr/bin/env python3
"""
Compression Detector
Auto-detects compression type and parameters from incoming data
"""

import logging
import struct
import zlib
import gzip
import lzma
from typing import Dict, Any, Optional, Tuple
from enum import Enum
import magic
import numpy as np

class CompressionType(Enum):
    """Supported compression types"""
    UNCOMPRESSED = "uncompressed"
    RLE = "rle"  # Run-Length Encoding
    DELTA = "delta"
    ZLIB = "zlib"
    GZIP = "gzip"
    LZMA = "lzma"
    UNKNOWN = "unknown"

class CompressionDetector:
    """Detects compression type and parameters from data"""
    
    def __init__(self):
        self.logger = logging.getLogger('compression_detector')
        
        # Compression signatures (magic numbers)
        self.compression_signatures = {
            b'\x1f\x8b\x08': CompressionType.GZIP,
            b'\x78\x9c': CompressionType.ZLIB,
            b'\x78\xda': CompressionType.ZLIB,
            b'\xfd7zXZ': CompressionType.LZMA,
            b'RLE': CompressionType.RLE,
            b'DELTA': CompressionType.DELTA,
        }
        
        # Statistics
        self.detection_stats = {
            'total_detections': 0,
            'successful_detections': 0,
            'failed_detections': 0,
            'type_counts': {comp_type.value: 0 for comp_type in CompressionType}
        }
    
    def detect_compression(self, data: bytes, metadata: Dict = None) -> Dict[str, Any]:
        """
        Detect compression type and parameters from data
        
        Args:
            data: Compressed data bytes
            metadata: Optional metadata about the data
            
        Returns:
            Dict with compression information
        """
        self.detection_stats['total_detections'] += 1
        
        try:
            # Method 1: Check magic numbers
            compression_type = self._detect_by_magic_numbers(data)
            
            # Method 2: Analyze data patterns if magic numbers don't work
            if compression_type == CompressionType.UNKNOWN:
                compression_type = self._detect_by_pattern_analysis(data)
            
            # Method 3: Use metadata if available
            if compression_type == CompressionType.UNKNOWN and metadata:
                compression_type = self._detect_from_metadata(metadata)
            
            # Calculate compression ratio if original size is known
            compression_ratio = self._calculate_compression_ratio(data, metadata)
            
            # Build result
            result = {
                'compression_type': compression_type.value,
                'confidence': self._calculate_confidence(data, compression_type),
                'compression_ratio': compression_ratio,
                'data_size': len(data),
                'estimated_original_size': self._estimate_original_size(data, compression_type),
                'recommended_decoder': self._get_recommended_decoder(compression_type)
            }
            
            # Update statistics
            self.detection_stats['successful_detections'] += 1
            self.detection_stats['type_counts'][compression_type.value] += 1
            
            self.logger.debug(f"Detected compression: {compression_type.value} "
                            f"(confidence: {result['confidence']:.2f})")
            
            return result
            
        except Exception as e:
            self.detection_stats['failed_detections'] += 1
            self.logger.error(f"Compression detection failed: {e}")
            
            return {
                'compression_type': CompressionType.UNKNOWN.value,
                'confidence': 0.0,
                'compression_ratio': 1.0,
                'data_size': len(data),
                'estimated_original_size': len(data),
                'error': str(e)
            }
    
    def _detect_by_magic_numbers(self, data: bytes) -> CompressionType:
        """Detect compression type using magic number signatures"""
        for signature, comp_type in self.compression_signatures.items():
            if data.startswith(signature):
                return comp_type
        
        # Check for custom RLE format (simple byte pattern)
        if self._looks_like_rle(data):
            return CompressionType.RLE
        
        # Check for delta encoding (specific patterns)
        if self._looks_like_delta(data):
            return CompressionType.DELTA
        
        return CompressionType.UNKNOWN
    
    def _detect_by_pattern_analysis(self, data: bytes) -> CompressionType:
        """Analyze data patterns to detect compression type"""
        if len(data) < 100:  # Too small for reliable pattern analysis
            return CompressionType.UNCOMPRESSED
        
        # Calculate entropy (compressed data typically has higher entropy)
        entropy = self._calculate_entropy(data)
        
        # Check for repeated patterns (RLE)
        repetition_ratio = self._calculate_repetition_ratio(data)
        
        # Analyze byte value distribution
        byte_distribution = self._analyze_byte_distribution(data)
        
        # Decision logic based on analysis
        if repetition_ratio > 0.3:
            return CompressionType.RLE
        elif entropy > 7.5:  # High entropy suggests compression
            if self._has_structured_pattern(data):
                return CompressionType.DELTA
            else:
                return CompressionType.ZLIB  # Most common general compression
        else:
            return CompressionType.UNCOMPRESSED
    
    def _detect_from_metadata(self, metadata: Dict) -> CompressionType:
        """Extract compression type from metadata"""
        comp_type_str = metadata.get('compression_type', '').lower()
        
        for comp_type in CompressionType:
            if comp_type.value == comp_type_str:
                return comp_type
        
        return CompressionType.UNKNOWN
    
    def _looks_like_rle(self, data: bytes) -> bool:
        """Check if data looks like Run-Length Encoding"""
        if len(data) < 8:
            return False
        
        # RLE often has repeating byte patterns: [count, value, count, value, ...]
        # Check for alternating high and low byte values
        byte_pairs = [(data[i], data[i+1]) for i in range(0, len(data)-1, 2)]
        
        # Count how many pairs look like (count, value)
        rle_like_pairs = 0
        for count_byte, value_byte in byte_pairs[:20]:  # Check first 20 pairs
            # Count byte should be small, value byte can be anything
            if count_byte < 128 and count_byte > 0:
                rle_like_pairs += 1
        
        return rle_like_pairs / len(byte_pairs[:20]) > 0.6
    
    def _looks_like_delta(self, data: bytes) -> bool:
        """Check if data looks like delta encoding"""
        if len(data) < 12:
            return False
        
        # Delta encoding often has small differences between consecutive bytes
        differences = []
        for i in range(len(data) - 1):
            diff = abs(data[i] - data[i+1])
            differences.append(diff)
        
        # Delta encoded data typically has small differences
        small_differences = sum(1 for diff in differences if diff < 10)
        small_diff_ratio = small_differences / len(differences)
        
        return small_diff_ratio > 0.7
    
    def _calculate_entropy(self, data: bytes) -> float:
        """Calculate Shannon entropy of the data"""
        if not data:
            return 0.0
        
        byte_counts = np.zeros(256, dtype=np.int32)
        for byte in data:
            byte_counts[byte] += 1
        
        probabilities = byte_counts[byte_counts > 0] / len(data)
        entropy = -np.sum(probabilities * np.log2(probabilities))
        
        return entropy
    
    def _calculate_repetition_ratio(self, data: bytes) -> float:
        """Calculate ratio of repeated byte sequences"""
        if len(data) < 4:
            return 0.0
        
        # Count repeated 2-byte sequences
        sequence_counts = {}
        for i in range(len(data) - 1):
            sequence = data[i:i+2]
            sequence_counts[sequence] = sequence_counts.get(sequence, 0) + 1
        
        # Calculate repetition ratio
        total_sequences = len(data) - 1
        repeated_sequences = sum(1 for count in sequence_counts.values() if count > 1)
        
        return repeated_sequences / total_sequences if total_sequences > 0 else 0.0
    
    def _analyze_byte_distribution(self, data: bytes) -> Dict[str, float]:
        """Analyze byte value distribution"""
        byte_array = np.frombuffer(data, dtype=np.uint8)
        
        return {
            'mean': float(np.mean(byte_array)),
            'std': float(np.std(byte_array)),
            'min': float(np.min(byte_array)),
            'max': float(np.max(byte_array)),
            'unique_bytes': len(np.unique(byte_array)) / len(byte_array)
        }
    
    def _has_structured_pattern(self, data: bytes) -> bool:
        """Check if data has structured patterns (suggesting delta encoding)"""
        # Look for regular patterns in the data
        if len(data) < 20:
            return False
        
        # Check for alternating patterns
        pattern_changes = 0
        for i in range(10, min(100, len(data))):
            if data[i] != data[i-10]:
                pattern_changes += 1
        
        return pattern_changes < 5  # Few changes suggest structured data
    
    def _calculate_compression_ratio(self, data: bytes, metadata: Dict = None) -> float:
        """Calculate compression ratio if original size is known"""
        if metadata and 'original_size' in metadata:
            original_size = metadata['original_size']
            if original_size > 0:
                return len(data) / original_size
        
        # Estimate based on data characteristics
        return self._estimate_compression_ratio(data)
    
    def _estimate_compression_ratio(self, data: bytes) -> float:
        """Estimate compression ratio based on data analysis"""
        entropy = self._calculate_entropy(data)
        
        # Simple heuristic: higher entropy suggests better compression
        if entropy > 7.8:
            return 0.3  # Good compression
        elif entropy > 7.0:
            return 0.6  # Moderate compression
        else:
            return 0.9  # Poor compression
    
    def _estimate_original_size(self, data: bytes, compression_type: CompressionType) -> int:
        """Estimate original data size"""
        base_size = len(data)
        
        # Rough estimates based on compression type
        expansion_factors = {
            CompressionType.UNCOMPRESSED: 1.0,
            CompressionType.RLE: 2.0,
            CompressionType.DELTA: 1.5,
            CompressionType.ZLIB: 3.0,
            CompressionType.GZIP: 3.0,
            CompressionType.LZMA: 4.0,
            CompressionType.UNKNOWN: 1.2
        }
        
        return int(base_size * expansion_factors.get(compression_type, 1.2))
    
    def _calculate_confidence(self, data: bytes, compression_type: CompressionType) -> float:
        """Calculate confidence score for detection"""
        if compression_type == CompressionType.UNKNOWN:
            return 0.1
        
        # Base confidence
        confidence = 0.7
        
        # Increase confidence based on specific checks
        if compression_type == CompressionType.RLE and self._looks_like_rle(data):
            confidence += 0.2
        
        if compression_type == CompressionType.DELTA and self._looks_like_delta(data):
            confidence += 0.2
        
        # Magic number matches give high confidence
        for signature, comp_type in self.compression_signatures.items():
            if data.startswith(signature) and comp_type == compression_type:
                confidence = 0.95
                break
        
        return min(confidence, 1.0)
    
    def _get_recommended_decoder(self, compression_type: CompressionType) -> str:
        """Get recommended decoder module for the compression type"""
        decoder_map = {
            CompressionType.UNCOMPRESSED: "raw_processor",
            CompressionType.RLE: "run_length_decoder",
            CompressionType.DELTA: "delta_decoder", 
            CompressionType.ZLIB: "zlib_decompressor",
            CompressionType.GZIP: "gzip_decompressor",
            CompressionType.LZMA: "lzma_decompressor",
            CompressionType.UNKNOWN: "fallback_decoder"
        }
        return decoder_map.get(compression_type, "fallback_decoder")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get detection statistics"""
        return self.detection_stats.copy()

# Utility function for quick detection
def detect_compression_quick(data: bytes) -> str:
    """Quick compression detection for simple cases"""
    detector = CompressionDetector()
    result = detector.detect_compression(data)
    return result['compression_type']

# Example usage
if __name__ == "__main__":
    # Test with sample data
    test_data = b"Hello World! " * 100  # Repeated data (good for RLE)
    
    detector = CompressionDetector()
    result = detector.detect_compression(test_data)
    
    print("Compression Detection Result:")
    for key, value in result.items():
        print(f"  {key}: {value}")
    
    print("\nDetection Statistics:")
    stats = detector.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")