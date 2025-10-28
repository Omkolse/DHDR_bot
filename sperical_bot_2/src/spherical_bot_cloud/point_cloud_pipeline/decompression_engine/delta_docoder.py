#!/usr/bin/env python3
"""
Delta Decoder
Decompresses delta-encoded point cloud data
Optimized for smooth value sequences like depth maps
"""

import logging
import struct
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import zlib

class DeltaDecodeResult(Enum):
    SUCCESS = "success"
    INVALID_FORMAT = "invalid_format"
    CORRUPTED_DATA = "corrupted_data"
    SIZE_MISMATCH = "size_mismatch"

@dataclass
class DeltaDecodeReport:
    """Delta decoding report container"""
    result: DeltaDecodeResult
    original_size: int
    decompressed_size: int
    compression_ratio: float
    processing_time_ms: float
    reconstruction_error: float
    metadata: Dict[str, Any]

class DeltaDecoder:
    """
    Delta Encoding Decoder for point cloud data
    Specialized for depth maps and sequential data with small differences
    """
    
    def __init__(self):
        self.logger = logging.getLogger('delta_decoder')
        
        # Supported delta encoding schemes
        self.supported_schemes = {
            'simple_delta': self._decode_simple_delta,
            'predictive_delta': self._decode_predictive_delta,
            'second_order_delta': self._decode_second_order_delta,
            'adaptive_delta': self._decode_adaptive_delta,
            'block_delta': self._decode_block_delta
        }
        
        # Configuration
        self.config = {
            'max_decompressed_size': 100 * 1024 * 1024,  # 100MB
            'enable_error_correction': True,
            'enable_validation': True,
            'smoothing_enabled': True,
            'smoothing_factor': 0.1
        }
        
        # Statistics
        self.stats = {
            'total_decodings': 0,
            'successful_decodings': 0,
            'failed_decodings': 0,
            'total_bytes_processed': 0,
            'average_compression_ratio': 0.0,
            'average_reconstruction_error': 0.0,
            'scheme_usage': {scheme: 0 for scheme in self.supported_schemes}
        }
    
    def decode(self, delta_data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], DeltaDecodeReport]:
        """
        Decode delta-encoded data
        
        Args:
            delta_data: Delta-encoded bytes
            metadata: Optional metadata about the encoding
            
        Returns:
            Tuple of (decompressed_data, decode_report)
        """
        import time
        start_time = time.time()
        
        self.stats['total_decodings'] += 1
        self.stats['total_bytes_processed'] += len(delta_data)
        
        try:
            # Determine delta scheme
            delta_scheme = self._detect_delta_scheme(delta_data, metadata)
            if delta_scheme not in self.supported_schemes:
                report = DeltaDecodeReport(
                    result=DeltaDecodeResult.INVALID_FORMAT,
                    original_size=0,
                    decompressed_size=0,
                    compression_ratio=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    reconstruction_error=0.0,
                    metadata={'detected_scheme': delta_scheme, 'error': 'Unsupported scheme'}
                )
                return None, report
            
            self.stats['scheme_usage'][delta_scheme] += 1
            
            # Validate input data
            if self.config['enable_validation']:
                validation_result = self._validate_delta_data(delta_data, delta_scheme, metadata)
                if not validation_result[0]:
                    report = DeltaDecodeReport(
                        result=DeltaDecodeResult.INVALID_FORMAT,
                        original_size=0,
                        decompressed_size=0,
                        compression_ratio=0.0,
                        processing_time_ms=(time.time() - start_time) * 1000,
                        reconstruction_error=0.0,
                        metadata={'error': validation_result[1]}
                    )
                    return None, report
            
            # Perform decoding
            decoder_func = self.supported_schemes[delta_scheme]
            decompressed_data, reconstruction_error = decoder_func(delta_data, metadata)
            
            if decompressed_data is None:
                report = DeltaDecodeReport(
                    result=DeltaDecodeResult.CORRUPTED_DATA,
                    original_size=0,
                    decompressed_size=0,
                    compression_ratio=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    reconstruction_error=0.0,
                    metadata={'scheme': delta_scheme}
                )
                return None, report
            
            # Calculate statistics
            original_size = len(delta_data)
            decompressed_size = len(decompressed_data)
            compression_ratio = original_size / decompressed_size if decompressed_size > 0 else 0.0
            
            # Update statistics
            self.stats['successful_decodings'] += 1
            self._update_average_ratio(compression_ratio)
            self._update_average_error(reconstruction_error)
            
            processing_time = (time.time() - start_time) * 1000
            
            self.logger.debug(f"Delta decoding successful: {delta_scheme}, "
                            f"ratio: {compression_ratio:.2f}, "
                            f"error: {reconstruction_error:.6f}, "
                            f"time: {processing_time:.2f}ms")
            
            report = DeltaDecodeReport(
                result=DeltaDecodeResult.SUCCESS,
                original_size=original_size,
                decompressed_size=decompressed_size,
                compression_ratio=compression_ratio,
                processing_time_ms=processing_time,
                reconstruction_error=reconstruction_error,
                metadata={
                    'scheme': delta_scheme,
                    'method': 'delta_decoding',
                    'error_correction': self.config['enable_error_correction'],
                    'smoothing': self.config['smoothing_enabled']
                }
            )
            
            return decompressed_data, report
            
        except Exception as e:
            self.stats['failed_decodings'] += 1
            self.logger.error(f"Delta decoding failed: {e}")
            
            report = DeltaDecodeReport(
                result=DeltaDecodeResult.CORRUPTED_DATA,
                original_size=0,
                decompressed_size=0,
                compression_ratio=0.0,
                processing_time_ms=(time.time() - start_time) * 1000,
                reconstruction_error=0.0,
                metadata={'error': str(e)}
            )
            
            return None, report
    
    def _detect_delta_scheme(self, data: bytes, metadata: Dict = None) -> str:
        """Detect delta encoding scheme from data and metadata"""
        if metadata and 'delta_scheme' in metadata:
            return metadata['delta_scheme']
        
        # Auto-detect based on data patterns and headers
        if len(data) < 4:
            return 'simple_delta'
        
        # Check for scheme signatures
        if data[:2] == b'PD':
            return 'predictive_delta'
        elif data[:2] == b'SD':
            return 'second_order_delta'
        elif data[:2] == b'AD':
            return 'adaptive_delta'
        elif data[:2] == b'BD':
            return 'block_delta'
        
        # Default to simple delta
        return 'simple_delta'
    
    def _validate_delta_data(self, data: bytes, scheme: str, metadata: Dict = None) -> Tuple[bool, str]:
        """Validate delta-encoded data integrity"""
        if len(data) < 2:
            return False, "Data too short for delta encoding"
        
        # Scheme-specific validation
        if scheme == 'predictive_delta':
            if data[:2] != b'PD':
                return False, "Invalid predictive delta signature"
        elif scheme == 'second_order_delta':
            if data[:2] != b'SD':
                return False, "Invalid second-order delta signature"
        elif scheme == 'adaptive_delta':
            if data[:2] != b'AD':
                return False, "Invalid adaptive delta signature"
        elif scheme == 'block_delta':
            if data[:2] != b'BD':
                return False, "Invalid block delta signature"
        
        # Check data size limits
        if len(data) > self.config['max_decompressed_size']:
            return False, f"Data too large: {len(data)} bytes"
        
        return True, "Valid"
    
    def _decode_simple_delta(self, data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], float]:
        """Decode simple first-order delta encoding"""
        try:
            if len(data) < 2:
                return None, 0.0
            
            # Simple delta: [base_value, delta1, delta2, ...]
            base_value = data[0]
            deltas = data[1:]
            
            decompressed = bytearray()
            decompressed.append(base_value)
            
            current_value = base_value
            total_error = 0.0
            
            for delta in deltas:
                # Handle signed delta (two's complement)
                if delta > 127:
                    delta = delta - 256
                
                current_value += delta
                current_value = max(0, min(255, current_value))  # Clamp to byte range
                
                decompressed.append(current_value & 0xFF)
                
                # Calculate reconstruction error (absolute difference from ideal)
                ideal_value = base_value + sum(deltas[:len(decompressed)-1])
                error = abs(current_value - ideal_value)
                total_error += error
            
            avg_error = total_error / len(decompressed) if decompressed else 0.0
            
            # Apply smoothing if enabled
            if self.config['smoothing_enabled'] and len(decompressed) > 10:
                decompressed = self._apply_temporal_smoothing(decompressed)
            
            return bytes(decompressed), avg_error
            
        except Exception as e:
            self.logger.error(f"Simple delta decoding error: {e}")
            return None, 0.0
    
    def _decode_predictive_delta(self, data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], float]:
        """Decode predictive delta encoding with error correction"""
        try:
            if len(data) < 8:
                return None, 0.0
            
            # Header: 'PD' + version + width + height
            version = data[2]
            width = struct.unpack('<H', data[3:5])[0]  # 16-bit width
            height = struct.unpack('<H', data[5:7])[0] # 16-bit height
            
            expected_size = width * height
            delta_data = data[7:]
            
            if len(delta_data) < expected_size:
                return None, 0.0
            
            decompressed = bytearray()
            total_error = 0.0
            
            # Start with base value (first pixel)
            base_value = delta_data[0]
            decompressed.append(base_value)
            
            # Decode row by row for 2D data
            index = 1
            for row in range(height):
                row_data = []
                
                if row == 0:
                    # First row: use left prediction
                    current = base_value
                    for col in range(1, width):
                        if index >= len(delta_data):
                            break
                        
                        delta = self._signed_byte_to_int(delta_data[index])
                        current += delta
                        current = max(0, min(255, current))
                        row_data.append(current)
                        index += 1
                    
                    decompressed.extend(row_data)
                else:
                    # Subsequent rows: use above prediction
                    for col in range(width):
                        if index >= len(delta_data):
                            break
                        
                        above_index = (row - 1) * width + col
                        above_value = decompressed[above_index]
                        
                        delta = self._signed_byte_to_int(delta_data[index])
                        current = above_value + delta
                        current = max(0, min(255, current))
                        
                        decompressed.append(current)
                        index += 1
            
            # Calculate average error
            avg_error = total_error / len(decompressed) if decompressed else 0.0
            
            return bytes(decompressed), avg_error
            
        except Exception as e:
            self.logger.error(f"Predictive delta decoding error: {e}")
            return None, 0.0
    
    def _decode_second_order_delta(self, data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], float]:
        """Decode second-order delta encoding (delta of deltas)"""
        try:
            if len(data) < 4:
                return None, 0.0
            
            # Second-order delta: [base, first_delta, second_delta1, second_delta2, ...]
            base = data[0]
            first_delta = self._signed_byte_to_int(data[1])
            
            decompressed = bytearray()
            decompressed.append(base)
            
            if len(data) > 2:
                current_value = base + first_delta
                current_value = max(0, min(255, current_value))
                decompressed.append(current_value)
                
                current_delta = first_delta
                total_error = 0.0
                
                for i in range(2, len(data)):
                    delta_of_delta = self._signed_byte_to_int(data[i])
                    current_delta += delta_of_delta
                    current_value += current_delta
                    current_value = max(0, min(255, current_value))
                    
                    decompressed.append(current_value & 0xFF)
            
            avg_error = total_error / len(decompressed) if decompressed else 0.0
            
            return bytes(decompressed), avg_error
            
        except Exception as e:
            self.logger.error(f"Second-order delta decoding error: {e}")
            return None, 0.0
    
    def _decode_adaptive_delta(self, data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], float]:
        """Decode adaptive delta encoding that switches precision"""
        try:
            if len(data) < 4:
                return None, 0.0
            
            decompressed = bytearray()
            total_error = 0.0
            
            i = 0
            precision = 1  # Start with 8-bit deltas
            base_value = data[0]
            decompressed.append(base_value)
            i += 1
            
            current_value = base_value
            
            while i < len(data):
                if precision == 1:
                    # 8-bit delta
                    if i >= len(data):
                        break
                    
                    delta = self._signed_byte_to_int(data[i])
                    i += 1
                    
                    # Check for precision change marker
                    if delta == -128:  # Special marker for precision change
                        precision = 2
                        continue
                
                elif precision == 2:
                    # 16-bit delta
                    if i + 1 >= len(data):
                        break
                    
                    delta = struct.unpack('<h', data[i:i+2])[0]  # Signed 16-bit
                    i += 2
                
                current_value += delta
                current_value = max(0, min(255, current_value))
                decompressed.append(current_value & 0xFF)
            
            avg_error = total_error / len(decompressed) if decompressed else 0.0
            
            return bytes(decompressed), avg_error
            
        except Exception as e:
            self.logger.error(f"Adaptive delta decoding error: {e}")
            return None, 0.0
    
    def _decode_block_delta(self, data: bytes, metadata: Dict = None) -> Tuple[Optional[bytes], float]:
        """Decode block-based delta encoding"""
        try:
            if len(data) < 10:
                return None, 0.0
            
            # Header: 'BD' + version + block_size + num_blocks
            version = data[2]
            block_size = struct.unpack('<H', data[3:5])[0]
            num_blocks = struct.unpack('<H', data[5:7])[0]
            
            decompressed = bytearray()
            total_error = 0.0
            i = 7
            
            for block_idx in range(num_blocks):
                if i >= len(data):
                    break
                
                # Block header: base_value + delta_scale
                base_value = data[i]
                delta_scale = data[i + 1]
                i += 2
                
                # Decode block deltas
                block_data = [base_value]
                current_value = base_value
                
                for j in range(block_size - 1):
                    if i >= len(data):
                        break
                    
                    delta = self._signed_byte_to_int(data[i])
                    scaled_delta = delta * delta_scale
                    
                    current_value += scaled_delta
                    current_value = max(0, min(255, current_value))
                    block_data.append(current_value & 0xFF)
                    
                    i += 1
                
                decompressed.extend(block_data)
            
            avg_error = total_error / len(decompressed) if decompressed else 0.0
            
            return bytes(decompressed), avg_error
            
        except Exception as e:
            self.logger.error(f"Block delta decoding error: {e}")
            return None, 0.0
    
    def _signed_byte_to_int(self, byte_val: int) -> int:
        """Convert byte to signed integer (-128 to 127)"""
        if byte_val > 127:
            return byte_val - 256
        return byte_val
    
    def _apply_temporal_smoothing(self, data: bytearray) -> bytearray:
        """Apply temporal smoothing to reduce noise in decoded data"""
        if len(data) < 3:
            return data
        
        smoothed = bytearray(data)
        alpha = self.config['smoothing_factor']
        
        for i in range(1, len(data) - 1):
            # Simple exponential smoothing
            smoothed[i] = int(
                alpha * data[i] + 
                (1 - alpha) * (smoothed[i-1] + data[i+1]) / 2
            )
        
        return smoothed
    
    def decode_to_numpy(self, delta_data: bytes, metadata: Dict = None) -> Optional[np.ndarray]:
        """Decode delta data directly to numpy array"""
        decompressed_data, report = self.decode(delta_data, metadata)
        
        if decompressed_data is None:
            return None
        
        try:
            # Determine array shape from metadata
            shape = None
            if metadata:
                if 'shape' in metadata:
                    shape = metadata['shape']
                elif 'width' in metadata and 'height' in metadata:
                    shape = (metadata['height'], metadata['width'])
            
            # Convert to numpy array
            array_data = np.frombuffer(decompressed_data, dtype=np.uint8)
            
            if shape and np.prod(shape) == len(array_data):
                array_data = array_data.reshape(shape)
            
            return array_data
            
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
    
    def _update_average_error(self, error: float):
        """Update average reconstruction error"""
        current_avg = self.stats['average_reconstruction_error']
        total_success = self.stats['successful_decodings']
        
        if total_success == 1:
            self.stats['average_reconstruction_error'] = error
        else:
            self.stats['average_reconstruction_error'] = (
                current_avg * (total_success - 1) + error
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

# Utility functions
def decode_delta_simple(data: bytes) -> Optional[bytes]:
    """Simple delta decoding"""
    decoder = DeltaDecoder()
    result, _ = decoder.decode(data)
    return result

def decode_delta_to_array(data: bytes, shape: Tuple = None) -> Optional[np.ndarray]:
    """Decode delta directly to numpy array"""
    decoder = DeltaDecoder()
    metadata = {'shape': shape} if shape else {}
    return decoder.decode_to_numpy(data, metadata)

# Example usage
if __name__ == "__main__":
    # Test with sample delta data (smooth sequence)
    test_data = bytes([100])  # Base value
    test_data += bytes([1, 1, 1, 1, -1, -1, -1, -1])  # Small deltas
    
    decoder = DeltaDecoder()
    result, report = decoder.decode(test_data)
    
    print("Delta Decoding Test:")
    print(f"Input: {list(test_data)}")
    print(f"Output: {list(result) if result else 'None'}")
    print(f"Report: {report.result.value}")
    print(f"Compression Ratio: {report.compression_ratio:.2f}")
    print(f"Reconstruction Error: {report.reconstruction_error:.6f}")
    
    print("\nDecoder Statistics:")
    stats = decoder.get_statistics()
    for key, value in stats.items():
        if key != 'config' and key != 'scheme_usage':
            print(f"  {key}: {value}")