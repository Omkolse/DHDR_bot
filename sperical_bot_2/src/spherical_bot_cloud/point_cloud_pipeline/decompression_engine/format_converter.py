#!/usr/bin/env python3
"""
Format Converter
Converts various point cloud formats to standard representations
Supports multiple input and output formats
"""

import logging
import struct
import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import Enum
import json
import base64

class PointCloudFormat(Enum):
    """Supported point cloud formats"""
    # Input formats
    RAW_BYTES = "raw_bytes"
    NUMPY_ARRAY = "numpy_array"
    PCD_ASCII = "pcd_ascii"
    PCD_BINARY = "pcd_binary"
    PLY_ASCII = "ply_ascii"
    PLY_BINARY = "ply_binary"
    LAS = "las"
    NPZ = "npz"
    JSON = "json"
    
    # Output formats
    NUMPY_XYZ = "numpy_xyz"
    NUMPY_XYZI = "numpy_xyzi"
    NUMPY_XYZRGB = "numpy_xyzrgb"
    PCD_ASCII_OUT = "pcd_ascii_out"
    PCD_BINARY_OUT = "pcd_binary_out"
    PLY_ASCII_OUT = "ply_ascii_out"
    PLY_BINARY_OUT = "ply_binary_out"
    JSON_OUT = "json_out"

@dataclass
class ConversionReport:
    """Format conversion report container"""
    result: str
    input_format: PointCloudFormat
    output_format: PointCloudFormat
    input_points: int
    output_points: int
    processing_time_ms: float
    data_loss: bool
    metadata_preserved: bool
    metadata: Dict[str, Any]

class FormatConverter:
    """
    Converts between various point cloud formats
    Handles format detection, validation, and transformation
    """
    
    def __init__(self):
        self.logger = logging.getLogger('format_converter')
        
        # Supported conversions
        self.supported_conversions = {
            # From RAW_BYTES
            PointCloudFormat.RAW_BYTES: [
                PointCloudFormat.NUMPY_XYZ,
                PointCloudFormat.NUMPY_XYZI,
                PointCloudFormat.PCD_ASCII_OUT,
                PointCloudFormat.JSON_OUT
            ],
            # From NUMPY_ARRAY
            PointCloudFormat.NUMPY_ARRAY: [
                PointCloudFormat.NUMPY_XYZ,
                PointCloudFormat.NUMPY_XYZI,
                PointCloudFormat.NUMPY_XYZRGB,
                PointCloudFormat.PCD_BINARY_OUT,
                PointCloudFormat.PLY_BINARY_OUT,
                PointCloudFormat.JSON_OUT
            ],
            # From PCD formats
            PointCloudFormat.PCD_ASCII: [PointCloudFormat.NUMPY_XYZ, PointCloudFormat.NUMPY_XYZI],
            PointCloudFormat.PCD_BINARY: [PointCloudFormat.NUMPY_XYZ, PointCloudFormat.NUMPY_XYZI],
            # From PLY formats
            PointCloudFormat.PLY_ASCII: [PointCloudFormat.NUMPY_XYZ, PointCloudFormat.NUMPY_XYZRGB],
            PointCloudFormat.PLY_BINARY: [PointCloudFormat.NUMPY_XYZ, PointCloudFormat.NUMPY_XYZRGB],
            # From JSON
            PointCloudFormat.JSON: [PointCloudFormat.NUMPY_XYZ, PointCloudFormat.NUMPY_XYZI],
        }
        
        # Configuration
        self.config = {
            'auto_detect_format': True,
            'validate_input': True,
            'preserve_metadata': True,
            'default_output_format': PointCloudFormat.NUMPY_XYZ,
            'max_points_for_ascii': 100000,  # Avoid huge ASCII files
            'compression_level': 6,  # For formats that support compression
        }
        
        # Statistics
        self.stats = {
            'total_conversions': 0,
            'successful_conversions': 0,
            'failed_conversions': 0,
            'format_usage': {fmt.value: 0 for fmt in PointCloudFormat},
            'total_points_converted': 0,
            'average_conversion_time_ms': 0.0
        }
    
    def convert(self, input_data: Any, 
                output_format: PointCloudFormat = None,
                input_format: PointCloudFormat = None,
                metadata: Dict = None) -> Tuple[Optional[Any], ConversionReport]:
        """
        Convert point cloud data between formats
        
        Args:
            input_data: Input point cloud data in various formats
            output_format: Desired output format
            input_format: Input format (auto-detected if None)
            metadata: Additional conversion metadata
            
        Returns:
            Tuple of (converted_data, conversion_report)
        """
        import time
        start_time = time.time()
        
        self.stats['total_conversions'] += 1
        
        try:
            # Detect input format if not provided
            if input_format is None and self.config['auto_detect_format']:
                input_format = self._detect_input_format(input_data, metadata)
            
            if input_format is None:
                report = ConversionReport(
                    result="format_detection_failed",
                    input_format=PointCloudFormat.RAW_BYTES,
                    output_format=output_format or self.config['default_output_format'],
                    input_points=0,
                    output_points=0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    data_loss=False,
                    metadata_preserved=False,
                    metadata={'error': 'Could not detect input format'}
                )
                return None, report
            
            # Determine output format
            if output_format is None:
                output_format = self.config['default_output_format']
            
            # Validate conversion support
            if not self._is_conversion_supported(input_format, output_format):
                report = ConversionReport(
                    result="unsupported_conversion",
                    input_format=input_format,
                    output_format=output_format,
                    input_points=0,
                    output_points=0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    data_loss=False,
                    metadata_preserved=False,
                    metadata={'error': f'Unsupported conversion: {input_format} -> {output_format}'}
                )
                return None, report
            
            # Validate input data
            if self.config['validate_input']:
                validation_result = self._validate_input_data(input_data, input_format, metadata)
                if not validation_result[0]:
                    report = ConversionReport(
                        result="invalid_input",
                        input_format=input_format,
                        output_format=output_format,
                        input_points=0,
                        output_points=0,
                        processing_time_ms=(time.time() - start_time) * 1000,
                        data_loss=False,
                        metadata_preserved=False,
                        metadata={'error': validation_result[1]}
                    )
                    return None, report
            
            # Get input point count
            input_points = self._get_point_count(input_data, input_format)
            self.stats['total_points_converted'] += input_points
            
            # Perform conversion
            converted_data = self._perform_conversion(input_data, input_format, output_format, metadata)
            
            if converted_data is None:
                report = ConversionReport(
                    result="conversion_failed",
                    input_format=input_format,
                    output_format=output_format,
                    input_points=input_points,
                    output_points=0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    data_loss=True,
                    metadata_preserved=False,
                    metadata={'input_format': input_format.value, 'output_format': output_format.value}
                )
                return None, report
            
            # Get output point count
            output_points = self._get_point_count(converted_data, output_format)
            
            # Check for data loss
            data_loss = (input_points != output_points)
            
            # Preserve metadata if requested
            metadata_preserved = False
            if self.config['preserve_metadata'] and metadata:
                metadata_preserved = self._preserve_metadata(converted_data, output_format, metadata)
            
            # Update statistics
            self.stats['successful_conversions'] += 1
            self.stats['format_usage'][input_format.value] += 1
            self.stats['format_usage'][output_format.value] += 1
            self._update_average_time((time.time() - start_time) * 1000)
            
            processing_time = (time.time() - start_time) * 1000
            
            self.logger.debug(f"Format conversion successful: {input_format.value} -> {output_format.value}, "
                            f"points: {input_points} -> {output_points}, "
                            f"time: {processing_time:.2f}ms")
            
            report = ConversionReport(
                result="success",
                input_format=input_format,
                output_format=output_format,
                input_points=input_points,
                output_points=output_points,
                processing_time_ms=processing_time,
                data_loss=data_loss,
                metadata_preserved=metadata_preserved,
                metadata={
                    'input_format': input_format.value,
                    'output_format': output_format.value,
                    'conversion_method': 'direct',
                    'data_loss_detected': data_loss
                }
            )
            
            return converted_data, report
            
        except Exception as e:
            self.stats['failed_conversions'] += 1
            self.logger.error(f"Format conversion failed: {e}")
            
            report = ConversionReport(
                result="error",
                input_format=input_format or PointCloudFormat.RAW_BYTES,
                output_format=output_format or self.config['default_output_format'],
                input_points=0,
                output_points=0,
                processing_time_ms=(time.time() - start_time) * 1000,
                data_loss=True,
                metadata_preserved=False,
                metadata={'error': str(e)}
            )
            
            return None, report
    
    def _detect_input_format(self, data: Any, metadata: Dict = None) -> Optional[PointCloudFormat]:
        """Auto-detect input data format"""
        if metadata and 'format' in metadata:
            format_str = metadata['format'].lower()
            for fmt in PointCloudFormat:
                if fmt.value == format_str:
                    return fmt
        
        # Detect based on data type and content
        if isinstance(data, np.ndarray):
            return PointCloudFormat.NUMPY_ARRAY
        
        elif isinstance(data, bytes):
            # Check for various binary formats
            if len(data) >= 4:
                # Check for PCD binary signature
                if data[:4] == b'VERSION .7':
                    return PointCloudFormat.PCD_BINARY
                # Check for PLY binary signature
                elif data[:3] == b'ply':
                    if b'binary' in data[:100]:
                        return PointCloudFormat.PLY_BINARY
                    else:
                        return PointCloudFormat.PLY_ASCII
                # Check for LAS signature (usually starts with "LASF")
                elif data[:4] == b'LASF':
                    return PointCloudFormat.LAS
            
            return PointCloudFormat.RAW_BYTES
        
        elif isinstance(data, str):
            # Check for ASCII formats
            if data.startswith('VERSION .7'):
                return PointCloudFormat.PCD_ASCII
            elif data.startswith('ply'):
                return PointCloudFormat.PLY_ASCII
            elif data.startswith('{') and data.endswith('}'):
                try:
                    json.loads(data)
                    return PointCloudFormat.JSON
                except:
                    pass
        
        elif isinstance(data, dict):
            return PointCloudFormat.JSON
        
        return None
    
    def _is_conversion_supported(self, input_format: PointCloudFormat, 
                                output_format: PointCloudFormat) -> bool:
        """Check if conversion is supported"""
        if input_format in self.supported_conversions:
            return output_format in self.supported_conversions[input_format]
        return False
    
    def _validate_input_data(self, data: Any, input_format: PointCloudFormat, 
                            metadata: Dict) -> Tuple[bool, str]:
        """Validate input data for the given format"""
        if input_format == PointCloudFormat.NUMPY_ARRAY:
            if not isinstance(data, np.ndarray):
                return False, "Expected numpy array"
            if data.ndim != 2:
                return False, "Expected 2D array (points x features)"
            if data.shape[1] not in [3, 4, 6, 7]:  # xyz, xyzi, xyzrgb, xyzrgba
                return False, f"Unexpected number of features: {data.shape[1]}"
        
        elif input_format == PointCloudFormat.RAW_BYTES:
            if not isinstance(data, (bytes, bytearray)):
                return False, "Expected bytes or bytearray"
            if len(data) < 12:  # Minimum for 1 point (3 floats)
                return False, "Data too short for point cloud"
        
        elif input_format == PointCloudFormat.JSON:
            if not isinstance(data, (str, dict)):
                return False, "Expected JSON string or dictionary"
        
        return True, "Valid"
    
    def _get_point_count(self, data: Any, data_format: PointCloudFormat) -> int:
        """Get number of points in the data"""
        try:
            if data_format in [PointCloudFormat.NUMPY_ARRAY, PointCloudFormat.NUMPY_XYZ, 
                              PointCloudFormat.NUMPY_XYZI, PointCloudFormat.NUMPY_XYZRGB]:
                return data.shape[0]
            
            elif data_format == PointCloudFormat.RAW_BYTES:
                # Assume 3 floats per point (x, y, z)
                return len(data) // 12
            
            elif data_format == PointCloudFormat.JSON:
                if isinstance(data, dict):
                    points = data.get('points', [])
                    return len(points)
                else:
                    import json
                    data_dict = json.loads(data)
                    points = data_dict.get('points', [])
                    return len(points)
            
            else:
                return 0
        except:
            return 0
    
    def _perform_conversion(self, input_data: Any, input_format: PointCloudFormat,
                          output_format: PointCloudFormat, metadata: Dict) -> Optional[Any]:
        """Perform the actual format conversion"""
        try:
            # First, convert to internal numpy representation
            if input_format == PointCloudFormat.NUMPY_ARRAY:
                points = input_data
            else:
                points = self._convert_to_numpy(input_data, input_format, metadata)
                if points is None:
                    return None
            
            # Then convert from numpy to desired output format
            return self._convert_from_numpy(points, output_format, metadata)
            
        except Exception as e:
            self.logger.error(f"Conversion error: {e}")
            return None
    
    def _convert_to_numpy(self, data: Any, input_format: PointCloudFormat, 
                         metadata: Dict) -> Optional[np.ndarray]:
        """Convert various formats to numpy array"""
        try:
            if input_format == PointCloudFormat.RAW_BYTES:
                return self._raw_bytes_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.PCD_ASCII:
                return self._pcd_ascii_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.PCD_BINARY:
                return self._pcd_binary_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.PLY_ASCII:
                return self._ply_ascii_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.PLY_BINARY:
                return self._ply_binary_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.JSON:
                return self._json_to_numpy(data, metadata)
            
            elif input_format == PointCloudFormat.LAS:
                return self._las_to_numpy(data, metadata)
            
            else:
                self.logger.error(f"Unsupported input format for numpy conversion: {input_format}")
                return None
                
        except Exception as e:
            self.logger.error(f"Failed to convert {input_format} to numpy: {e}")
            return None
    
    def _convert_from_numpy(self, points: np.ndarray, output_format: PointCloudFormat,
                           metadata: Dict) -> Optional[Any]:
        """Convert numpy array to various output formats"""
        try:
            if output_format == PointCloudFormat.NUMPY_XYZ:
                # Ensure we have at least XYZ
                if points.shape[1] >= 3:
                    return points[:, :3]
                else:
                    return points
            
            elif output_format == PointCloudFormat.NUMPY_XYZI:
                # XYZ + Intensity (add dummy intensity if needed)
                if points.shape[1] >= 4:
                    return points[:, :4]  # Assume 4th is intensity
                else:
                    # Add dummy intensity
                    xyz = points[:, :3]
                    intensity = np.ones((xyz.shape[0], 1))
                    return np.hstack([xyz, intensity])
            
            elif output_format == PointCloudFormat.NUMPY_XYZRGB:
                # XYZ + RGB (add dummy RGB if needed)
                if points.shape[1] >= 6:
                    return points[:, :6]  # Assume 4-6 are RGB
                else:
                    # Add dummy RGB
                    xyz = points[:, :3]
                    rgb = np.ones((xyz.shape[0], 3)) * 255  # White
                    return np.hstack([xyz, rgb])
            
            elif output_format == PointCloudFormat.PCD_ASCII_OUT:
                return self._numpy_to_pcd_ascii(points, metadata)
            
            elif output_format == PointCloudFormat.PCD_BINARY_OUT:
                return self._numpy_to_pcd_binary(points, metadata)
            
            elif output_format == PointCloudFormat.PLY_ASCII_OUT:
                return self._numpy_to_ply_ascii(points, metadata)
            
            elif output_format == PointCloudFormat.PLY_BINARY_OUT:
                return self._numpy_to_ply_binary(points, metadata)
            
            elif output_format == PointCloudFormat.JSON_OUT:
                return self._numpy_to_json(points, metadata)
            
            else:
                self.logger.error(f"Unsupported output format: {output_format}")
                return None
                
        except Exception as e:
            self.logger.error(f"Failed to convert numpy to {output_format}: {e}")
            return None
    
    def _raw_bytes_to_numpy(self, data: bytes, metadata: Dict) -> Optional[np.ndarray]:
        """Convert raw bytes to numpy array"""
        try:
            # Determine point structure from metadata
            point_size = metadata.get('point_size', 12)  # Default: 3 floats
            dtype = metadata.get('dtype', np.float32)
            
            # Calculate number of points
            num_points = len(data) // point_size
            
            # Convert to numpy
            points = np.frombuffer(data, dtype=dtype)
            points = points.reshape(num_points, -1)
            
            return points
            
        except Exception as e:
            self.logger.error(f"Raw bytes to numpy conversion failed: {e}")
            return None
    
    def _pcd_ascii_to_numpy(self, data: str, metadata: Dict) -> Optional[np.ndarray]:
        """Convert PCD ASCII to numpy array"""
        try:
            lines = data.split('\n')
            points_start = False
            points_data = []
            
            for line in lines:
                if line.startswith('DATA ascii'):
                    points_start = True
                    continue
                
                if points_start and line.strip():
                    # Parse point data
                    values = line.strip().split()
                    if len(values) >= 3:  # At least XYZ
                        point = [float(v) for v in values[:3]]
                        if len(values) >= 4:  # Intensity
                            point.append(float(values[3]))
                        if len(values) >= 7:  # RGB
                            point.extend([float(v) for v in values[4:7]])
                        points_data.append(point)
            
            return np.array(points_data)
            
        except Exception as e:
            self.logger.error(f"PCD ASCII to numpy conversion failed: {e}")
            return None
    
    def _pcd_binary_to_numpy(self, data: bytes, metadata: Dict) -> Optional[np.ndarray]:
        """Convert PCD binary to numpy array (simplified)"""
        try:
            # This is a simplified implementation
            # Full PCD binary parsing would be more complex
            header_end = data.find(b'DATA binary')
            if header_end == -1:
                return None
            
            # Skip to data section
            data_start = header_end + len('DATA binary') + 1
            point_data = data[data_start:]
            
            # Assume 4 floats per point (XYZ + intensity)
            num_points = len(point_data) // 16
            points = np.frombuffer(point_data, dtype=np.float32, count=num_points * 4)
            points = points.reshape(num_points, 4)
            
            return points
            
        except Exception as e:
            self.logger.error(f"PCD binary to numpy conversion failed: {e}")
            return None
    
    def _json_to_numpy(self, data: Union[str, dict], metadata: Dict) -> Optional[np.ndarray]:
        """Convert JSON to numpy array"""
        try:
            if isinstance(data, str):
                import json
                data_dict = json.loads(data)
            else:
                data_dict = data
            
            points = data_dict.get('points', [])
            if not points:
                return None
            
            # Convert list of points to numpy
            return np.array(points)
            
        except Exception as e:
            self.logger.error(f"JSON to numpy conversion failed: {e}")
            return None
    
    def _numpy_to_pcd_ascii(self, points: np.ndarray, metadata: Dict) -> str:
        """Convert numpy array to PCD ASCII format"""
        try:
            header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z"""
            
            if points.shape[1] >= 4:
                header += " intensity"
            if points.shape[1] >= 6:
                header += " rgb"
            
            header += f"""
SIZE 4 4 4"""
            if points.shape[1] >= 4:
                header += " 4"
            if points.shape[1] >= 6:
                header += " 4"
            
            header += f"""
TYPE F F F"""
            if points.shape[1] >= 4:
                header += " F"
            if points.shape[1] >= 6:
                header += " F"
            
            header += f"""
COUNT 1 1 1"""
            if points.shape[1] >= 4:
                header += " 1"
            if points.shape[1] >= 6:
                header += " 1"
            
            header += f"""
WIDTH {points.shape[0]}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points.shape[0]}
DATA ascii
"""
            
            # Add point data
            point_lines = []
            for i in range(points.shape[0]):
                point_str = " ".join(f"{val:.6f}" for val in points[i])
                point_lines.append(point_str)
            
            return header + "\n".join(point_lines)
            
        except Exception as e:
            self.logger.error(f"Numpy to PCD ASCII conversion failed: {e}")
            return None
    
    def _numpy_to_json(self, points: np.ndarray, metadata: Dict) -> str:
        """Convert numpy array to JSON format"""
        try:
            result = {
                'points': points.tolist(),
                'metadata': {
                    'point_count': points.shape[0],
                    'dimensions': points.shape[1],
                    'conversion_timestamp': np.datetime64('now').astype(str)
                }
            }
            
            # Add original metadata if available
            if metadata:
                result['original_metadata'] = metadata
            
            import json
            return json.dumps(result, indent=2)
            
        except Exception as e:
            self.logger.error(f"Numpy to JSON conversion failed: {e}")
            return None
    
    def _numpy_to_pcd_binary(self, points: np.ndarray, metadata: Dict) -> bytes:
        """Convert numpy array to PCD binary format (simplified)"""
        try:
            # Create ASCII header
            ascii_header = self._numpy_to_pcd_ascii(points, metadata)
            if ascii_header is None:
                return None
            
            # Replace DATA ascii with DATA binary
            binary_header = ascii_header.replace('DATA ascii', 'DATA binary') + '\n'
            
            # Convert points to bytes
            points_bytes = points.astype(np.float32).tobytes()
            
            return binary_header.encode('utf-8') + points_bytes
            
        except Exception as e:
            self.logger.error(f"Numpy to PCD binary conversion failed: {e}")
            return None
    
    # Placeholder methods for other formats
    def _ply_ascii_to_numpy(self, data: str, metadata: Dict) -> Optional[np.ndarray]:
        self.logger.warning("PLY ASCII to numpy conversion not fully implemented")
        return None
    
    def _ply_binary_to_numpy(self, data: bytes, metadata: Dict) -> Optional[np.ndarray]:
        self.logger.warning("PLY binary to numpy conversion not fully implemented")
        return None
    
    def _las_to_numpy(self, data: bytes, metadata: Dict) -> Optional[np.ndarray]:
        self.logger.warning("LAS to numpy conversion not fully implemented")
        return None
    
    def _numpy_to_ply_ascii(self, points: np.ndarray, metadata: Dict) -> str:
        self.logger.warning("Numpy to PLY ASCII conversion not fully implemented")
        return None
    
    def _numpy_to_ply_binary(self, points: np.ndarray, metadata: Dict) -> bytes:
        self.logger.warning("Numpy to PLY binary conversion not fully implemented")
        return None
    
    def _preserve_metadata(self, converted_data: Any, output_format: PointCloudFormat,
                          metadata: Dict) -> bool:
        """Attempt to preserve metadata in the converted data"""
        # This would be format-specific metadata preservation
        # For now, we just return whether metadata was available
        return metadata is not None and len(metadata) > 0
    
    def _update_average_time(self, time_ms: float):
        """Update average conversion time"""
        current_avg = self.stats['average_conversion_time_ms']
        total_success = self.stats['successful_conversions']
        
        if total_success == 1:
            self.stats['average_conversion_time_ms'] = time_ms
        else:
            self.stats['average_conversion_time_ms'] = (
                current_avg * (total_success - 1) + time_ms
            ) / total_success
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get converter statistics"""
        success_rate = (
            self.stats['successful_conversions'] / self.stats['total_conversions'] 
            if self.stats['total_conversions'] > 0 else 0.0
        )
        
        return {
            **self.stats,
            'success_rate': success_rate,
            'config': {
                'auto_detect_format': self.config['auto_detect_format'],
                'default_output_format': self.config['default_output_format'].value
            }
        }

# Utility functions
def convert_format(input_data: Any, output_format: PointCloudFormat) -> Optional[Any]:
    """Simple format conversion"""
    converter = FormatConverter()
    result, _ = converter.convert(input_data, output_format)
    return result

def detect_format(data: Any) -> Optional[PointCloudFormat]:
    """Detect data format"""
    converter = FormatConverter()
    return converter._detect_input_format(data)

# Example usage
if __name__ == "__main__":
    # Test with sample numpy data
    test_points = np.random.rand(100, 3).astype(np.float32)
    
    converter = FormatConverter()
    result, report = converter.convert(test_points, PointCloudFormat.JSON_OUT)
    
    print("Format Conversion Test:")
    print(f"Input shape: {test_points.shape}")
    print(f"Input format: {report.input_format.value}")
    print(f"Output format: {report.output_format.value}")
    print(f"Points converted: {report.input_points} -> {report.output_points}")
    print(f"Data loss: {report.data_loss}")
    
    if result and len(str(result)) < 500:  # Don't print huge results
        print(f"Output sample: {str(result)[:200]}...")
    
    print("\nConverter Statistics:")
    stats = converter.get_statistics()
    for key, value in stats.items():
        if key != 'config' and key != 'format_usage':
            print(f"  {key}: {value}")