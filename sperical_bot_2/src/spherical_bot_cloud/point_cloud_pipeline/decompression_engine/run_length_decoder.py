#!/usr/bin/env python3
"""
Run-Length Decoder
Decompresses RLE-encoded point cloud data
"""

import logging
import struct
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import zlib

class RLEDecodeResult(Enum):
    SUCCESS = "success"
    INVALID_FORMAT = "invalid_format"
    CORRUPTED_DATA = "corrupted_data"
    UNSUPPORTED_VERSION = "unsupported_version"

@dataclass
class RLEDecodeReport:
    """RL decoding report container"""
    result: RLEDecodeResult
    original_size: int
    decompressed_size: int
    compression_ratio: float
    processing_time_ms: float
    metadata: Dict[str, Any]

class RunLengthDecoder:
    """
    Run-Length Encoding Decoder for point cloud data
    Supports multiple RLE formats and optimizations
    """
    
    def __init__(self):
        self.logger = logging.getLogger('rle_decoder')
        
        # Supported RLE formats
        self.supported_formats = {
            'standard_rle': self._decode_standard_rle,
            'packed_rle': self._decode_packed_rle,
            'differential_rle': self._decode_differential_rle,
            'adaptive_rle': self._decode_adaptive_rle
        }
        
        # Configuration
        self.config = {
            'max_decompressed_size': 100 * 1024 * 1024,  # 100MB
            'max_run_length': 65535,
            'enable_validation': True,
            'enable_optimization': True
        }
        
        # Statistics
        self.stats = {
            'total_decodings': 0,
            'successful_decodings': 0,
            'failed_decodings': 0,
            'total_bytes_processed': 0,
            'average_compression_ratio': 0.0,
            'format_usage': {fmt: 0 for fmt in self.supported_formats}
        }
    
    def decode(self, compressed_data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], RLEDecodeReport]:
        """
        Decode RLE-compressed data
        
        Args:
            compressed_data: RLE-compressed bytes
            metadata: Optional metadata about the compression
            
        Returns:
            Tuple of (decompressed_data, decode_report)
        """
        import time
        start_time = time.time()
        
        self.stats['total_decodings'] += 1
        self.stats['total_bytes_processed'] += len(compressed_data)
        
        try:
            # Determine RLE format
            rle_format = self._detect_rle_format(compressed_data, metadata)
            if rle_format not in self.supported_formats:
                report = RLEDecodeReport(
                    result=RLEDecodeResult.UNSUPPORTED_VERSION,
                    original_size=0,
                    decompressed_size=0,
                    compression_ratio=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'detected_format': rle_format, 'error': 'Unsupported format'}
                )
                return None, report
            
            self.stats['format_usage'][rle_format] += 1
            
            # Validate input data
            if self.config['enable_validation']:
                validation_result = self._validate_rle_data(compressed_data, rle_format)
                if not validation_result[0]:
                    report = RLEDecodeReport(
                        result=RLEDecodeResult.INVALID_FORMAT,
                        original_size=0,
                        decompressed_size=0,
                        compression_ratio=0.0,
                        processing_time_ms=(time.time() - start_time) * 1000,
                        metadata={'error': validation_result[1]}
                    )
                    return None, report
            
            # Perform decoding
            decoder_func = self.supported_formats[rle_format]
            decompressed_data = decoder_func(compressed_data, metadata)
            
            if decompressed_data is None:
                report = RLEDecodeReport(
                    result=RLEDecodeResult.CORRUPTED_DATA,
                    original_size=0,
                    decompressed_size=0,
                    compression_ratio=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'format': rle_format}
                )
                return None, report
            
            # Calculate statistics
            original_size = len(compressed_data)
            decompressed_size = len(decompressed_data)
            compression_ratio = original_size / decompressed_size if decompressed_size > 0 else 0.0
            
            # Update statistics
            self.stats['successful_decodings'] += 1
            self._update_average_ratio(compression_ratio)
            
            processing_time = (time.time() - start_time) * 1000
            
            self.logger.debug(f"RLE decoding successful: {rle_format}, "
                            f"ratio: {compression_ratio:.2f}, "
                            f"time: {processing_time:.2f}ms")
            
            report = RLEDecodeReport(
                result=RLEDecodeResult.SUCCESS,
                original_size=original_size,
                decompressed_size=decompressed_size,
                compression_ratio=compression_ratio,
                processing_time_ms=processing_time,
                metadata={
                    'format': rle_format,
                    'method': 'rle_decoding',
                    'optimizations_applied': self.config['enable_optimization']
                }
            )
            
            return decompressed_data, report
            
        except Exception as e:
            self.stats['failed_decodings'] += 1
            self.logger.error(f"RLE decoding failed: {e}")
            
            report = RLEDecodeReport(
                result=RLEDecodeResult.CORRUPTED_DATA,
                original_size=0,
                decompressed_size=0,
                compression_ratio=0.0,
                processing_time_ms=(time.time() - start_time) * 1000,
                metadata={'error': str(e)}
            )
            
            return None, report
    
    def _detect_rle_format(self, data: bytes, metadata: Dict = None) -> str:
        """Detect RLE format from data and metadata"""
        if metadata and 'rle_format' in metadata:
            return metadata['rle_format']
        
        # Auto-detect based on data patterns
        if len(data) < 4:
            return 'standard_rle'
        
        # Check for packed RLE signature
        if data[:2] == b'PR':
            return 'packed_rle'
        
        # Check for differential RLE signature
        if data[:2] == b'DR':
            return 'differential_rle'
        
        # Check for adaptive RLE signature
        if data[:2] == b'AR':
            return 'adaptive_rle'
        
        # Default to standard RLE
        return 'standard_rle'
    
    def _validate_rle_data(self, data: bytes, rle_format: str) -> Tuple[bool, str]:
        """Validate RLE data integrity"""
        if len(data) < 4:
            return False, "Data too short for RLE"
        
        if rle_format == 'packed_rle':
            if data[:2] != b'PR':
                return False, "Invalid packed RLE signature"
            version = data[2]
            if version != 1:
                return False, f"Unsupported packed RLE version: {version}"
        
        elif rle_format == 'differential_rle':
            if data[:2] != b'DR':
                return False, "Invalid differential RLE signature"
        
        elif rle_format == 'adaptive_rle':
            if data[:2] != b'AR':
                return False, "Invalid adaptive RLE signature"
        
        # Check for reasonable data size
        if len(data) > self.config['max_decompressed_size']:
            return False, f"Data too large: {len(data)} bytes"
        
        return True, "Valid"
    
    def _decode_standard_rle(self, data: bytes, metadata: Dict = None) -> Optional[bytes]:
        """Decode standard RLE format: [count, value, count, value, ...]"""
        try:
            decompressed = bytearray()
            i = 0
            
            while i < len(data):
                if i + 1 >= len(data):
                    break
                
                count = data[i]
                value = data[i + 1]
                
                # Validate run length
                if count == 0:
                    self.logger.warning("Zero run length encountered")
                    i += 2
                    continue
                
                if count > self.config['max_run_length']:
                    self.logger.error(f"Run length too large: {count}")
                    return None
                
                # Add repeated value to output
                decompressed.extend([value] * count)
                i += 2
            
            # Apply optimizations if enabled
            if self.config['enable_optimization']:
                decompressed = self._optimize_decompressed_data(decompressed, metadata)
            
            return bytes(decompressed)
            
        except Exception as e:
            self.logger.error(f"Standard RLE decoding error: {e}")
            return None
    
    def _decode_packed_rle(self, data: bytes, metadata: Dict = None) -> Optional[bytes]:
        """Decode packed RLE format with header and optimized encoding"""
        try:
            # Parse header: 'PR' + version + original_size
            if len(data) < 8:
                return None
            
            version = data[2]
            original_size = struct.unpack('<I', data[3:7])[0]  # Little-endian 32-bit
            
            decompressed = bytearray()
            i = 7  # Start after header
            
            while i < len(data):
                if i + 1 >= len(data):
                    break
                
                # Packed format: high bit indicates extended count
                count_byte = data[i]
                value = data[i + 1]
                
                if count_byte & 0x80:  # Extended count
                    if i + 2 >= len(data):
                        break
                    count = ((count_byte & 0x7F) << 8) | data[i + 2]
                    i += 3
                else:
                    count = count_byte
                    i += 2
                
                # Validate run length
                if count == 0:
                    continue
                
                if count > self.config['max_run_length']:
                    self.logger.error(f"Run length too large: {count}")
                    return None
                
                decompressed.extend([value] * count)
            
            # Verify we got the expected size
            if len(decompressed) != original_size:
                self.logger.warning(f"Size mismatch: expected {original_size}, got {len(decompressed)}")
            
            return bytes(decompressed)
            
        except Exception as e:
            self.logger.error(f"Packed RLE decoding error: {e}")
            return None
    
    def _decode_differential_rle(self, data: bytes, metadata: Dict = None) -> Optional[bytes]:
        """Decode differential RLE for smooth value changes"""
        try:
            if len(data) < 3:
                return None
            
            decompressed = bytearray()
            i = 0
            
            # Initial value
            current_value = data[0]
            i += 1
            
            while i < len(data):
                if i + 1 >= len(data):
                    break
                
                count = data[i]
                delta = data[i + 1]
                
                # Handle signed delta (two's complement)
                if delta > 127:
                    delta = delta - 256
                
                # Apply run
                for _ in range(count):
                    decompressed.append(current_value)
                    current_value = (current_value + delta) & 0xFF  # Wrap around
                
                i += 2
            
            return bytes(decompressed)
            
        except Exception as e:
            self.logger.error(f"Differential RLE decoding error: {e}")
            return None
    
    def _decode_adaptive_rle(self, data: bytes, metadata: Dict = None) -> Optional[bytes]:
        """Decode adaptive RLE that switches between modes"""
        try:
            if len(data) < 4:
                return None
            
            decompressed = bytearray()
            i = 0
            mode = 'standard'  # Start with standard mode
            
            while i < len(data):
                if mode == 'standard':
                    # Standard RLE mode
                    if i + 1 >= len(data):
                        break
                    
                    count = data[i]
                    value = data[i + 1]
                    
                    if count == 0:  # Mode switch marker
                        mode = 'raw'
                        i += 2
                        continue
                    
                    decompressed.extend([value] * count)
                    i += 2
                
                elif mode == 'raw':
                    # Raw data mode
                    if i >= len(data):
                        break
                    
                    length = data[i]
                    i += 1
                    
                    if i + length > len(data):
                        length = len(data) - i
                    
                    decompressed.extend(data[i:i + length])
                    i += length
                    
                    mode = 'standard'  # Switch back to standard mode
            
            return bytes(decompressed)
            
        except Exception as e:
            self.logger.error(f"Adaptive RLE decoding error: {e}")
            return None
    
    def _optimize_decompressed_data(self, data: bytes, metadata: Dict = None) -> bytes:
        """Apply optimizations to decompressed data"""
        # Simple optimization: remove trailing zeros if they dominate
        if len(data) > 1000:
            # Count trailing zeros
            trailing_zeros = 0
            for i in range(len(data) - 1, max(len(data) - 1000, 0), -1):
                if data[i] == 0:
                    trailing_zeros += 1
                else:
                    break
            
            # If more than 80% are zeros in the trailing section, trim them
            if trailing_zeros > 800:
                return data[:len(data) - trailing_zeros]
        
        return data
    
    def decode_to_numpy(self, compressed_data: bytes, metadata: Dict = None) -> Optional[np.ndarray]:
        """Decode RLE data directly to numpy array"""
        decompressed_data, report = self.decode(compressed_data, metadata)
        
        if decompressed_data is None:
            return None
        
        try:
            # Convert bytes to numpy array of appropriate dtype
            if metadata and 'dtype' in metadata:
                dtype = np.dtype(metadata['dtype'])
            else:
                # Auto-detect dtype
                dtype = np.uint8
            
            # Handle point cloud specific format
            if metadata and 'point_cloud_format' in metadata:
                fmt = metadata['point_cloud_format']
                if fmt == 'xyz':
                    # 3 floats per point (x, y, z)
                    points = np.frombuffer(decompressed_data, dtype=np.float32)
                    return points.reshape(-1, 3)
                elif fmt == 'xyzi':
                    # 4 floats per point (x, y, z, intensity)
                    points = np.frombuffer(decompressed_data, dtype=np.float32)
                    return points.reshape(-1, 4)
            
            # Default: 1D array of bytes
            return np.frombuffer(decompressed_data, dtype=dtype)
            
        except Exception as e:
            self.logger.error(f"Failed to convert to numpy array: {e}")
            return None
    
    def _update_average_ratio(self, ratio: float):
        """Update average compression ratio"""
        current_avg = self.stats['average_compression_ratio']
        total_success = self.stats['successful_decodings']
        
        if total_success == 1:
            self.stats['average_compression_ratio'] = ratio
        else:
            self.stats['average_compression_ratio'] = (
                current_avg * (total_success - 1) + ratio
            ) / total_success
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get decoder statistics"""
        success_rate = (
            self.stats['successful_decodings'] / self.stats['total_decodings'] 
            if self.stats['total_decodings'] > 0 else 0.0
        )
        
        return {
            **self.stats,
            'success_rate': success_rate,
            'config': self.config
        }
    
    def reset_statistics(self):
        """Reset decoder statistics"""
        self.stats = {
            'total_decodings': 0,
            'successful_decodings': 0,
            'failed_decodings': 0,
            'total_bytes_processed': 0,
            'average_compression_ratio': 0.0,
            'format_usage': {fmt: 0 for fmt in self.supported_formats}
        }

# Utility functions for common RLE operations
def decode_rle_simple(data: bytes) -> Optional[bytes]:
    """Simple RLE decoding for standard format"""
    decoder = RunLengthDecoder()
    result, _ = decoder.decode(data)
    return result

def decode_rle_to_array(data: bytes, dtype=np.uint8) -> Optional[np.ndarray]:
    """Decode RLE directly to numpy array"""
    decoder = RunLengthDecoder()
    metadata = {'dtype': dtype.name}
    return decoder.decode_to_numpy(data, metadata)

# Example usage and testing
if __name__ == "__main__":
    # Test with sample RLE data
    test_data = bytes([
        5, 65,    # 5 'A's
        3, 66,    # 3 'B's  
        2, 67,    # 2 'C's
        4, 65     # 4 'A's
    ])
    
    decoder = RunLengthDecoder()
    result, report = decoder.decode(test_data)
    
    print("RLE Decoding Test:")
    print(f"Input: {test_data}")
    print(f"Output: {result}")
    print(f"Report: {report.result.value}")
    print(f"Compression Ratio: {report.compression_ratio:.2f}")
    
    print("\nDecoder Statistics:")
    stats = decoder.get_statistics()
    for key, value in stats.items():
        if key != 'config' and key != 'format_usage':
            print(f"  {key}: {value}")