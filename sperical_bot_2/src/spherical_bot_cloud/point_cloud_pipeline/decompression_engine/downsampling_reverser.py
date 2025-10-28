#!/usr/bin/env python3
"""
Downsampling Reverser
Reconstructs original resolution from downsampled point cloud data
Uses interpolation and super-resolution techniques
"""

import logging
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import cv2
from scipy import interpolate
from scipy.ndimage import zoom

class ReconstructionMethod(Enum):
    NEAREST_NEIGHBOR = "nearest"
    BILINEAR = "bilinear"
    BICUBIC = "bicubic"
    LANCZOS = "lanczos"
    SUPER_RESOLUTION = "super_resolution"

@dataclass
class ReconstructionReport:
    """Downsampling reversal report container"""
    result: str
    original_shape: Tuple[int, ...]
    reconstructed_shape: Tuple[int, ...]
    scale_factor: Tuple[float, ...]
    method: ReconstructionMethod
    quality_score: float
    processing_time_ms: float
    metadata: Dict[str, Any]

class DownsamplingReverser:
    """
    Reverses downsampling operations on point cloud data
    Uses various interpolation and super-resolution techniques
    """
    
    def __init__(self):
        self.logger = logging.getLogger('downsampling_reverser')
        
        # Supported reconstruction methods
        self.supported_methods = {
            ReconstructionMethod.NEAREST_NEIGHBOR: self._reconstruct_nearest,
            ReconstructionMethod.BILINEAR: self._reconstruct_bilinear,
            ReconstructionMethod.BICUBIC: self._reconstruct_bicubic,
            ReconstructionMethod.LANCZOS: self._reconstruct_lanczos,
            ReconstructionMethod.SUPER_RESOLUTION: self._reconstruct_super_resolution
        }
        
        # Configuration
        self.config = {
            'max_scale_factor': 8.0,
            'min_scale_factor': 1.1,
            'default_method': ReconstructionMethod.BICUBIC,
            'enable_quality_assessment': True,
            'super_resolution_model_path': None,
            'max_reconstructed_points': 10_000_000  # 10 million points
        }
        
        # Statistics
        self.stats = {
            'total_reconstructions': 0,
            'successful_reconstructions': 0,
            'failed_reconstructions': 0,
            'average_quality_score': 0.0,
            'method_usage': {method.value: 0 for method in self.supported_methods},
            'total_points_processed': 0
        }
        
        # Load super-resolution model if available
        self.sr_model = None
        self._load_super_resolution_model()
    
    def reconstruct(self, downsampled_data: np.ndarray, 
                   target_shape: Tuple[int, ...],
                   method: ReconstructionMethod = None,
                   metadata: Dict = None) -> Tuple[Optional[np.ndarray], ReconstructionReport]:
        """
        Reconstruct original resolution from downsampled data
        
        Args:
            downsampled_data: Downsampled numpy array
            target_shape: Desired output shape
            method: Reconstruction method to use
            metadata: Optional metadata about the downsampling
            
        Returns:
            Tuple of (reconstructed_data, reconstruction_report)
        """
        import time
        start_time = time.time()
        
        self.stats['total_reconstructions'] += 1
        self.stats['total_points_processed'] += downsampled_data.size
        
        try:
            # Validate inputs
            validation_result = self._validate_inputs(downsampled_data, target_shape, metadata)
            if not validation_result[0]:
                report = ReconstructionReport(
                    result="invalid_input",
                    original_shape=downsampled_data.shape,
                    reconstructed_shape=target_shape,
                    scale_factor=(1.0, 1.0),
                    method=method or self.config['default_method'],
                    quality_score=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'error': validation_result[1]}
                )
                return None, report
            
            # Determine reconstruction method
            if method is None:
                method = self._select_optimal_method(downsampled_data, target_shape, metadata)
            
            if method not in self.supported_methods:
                report = ReconstructionReport(
                    result="unsupported_method",
                    original_shape=downsampled_data.shape,
                    reconstructed_shape=target_shape,
                    scale_factor=(1.0, 1.0),
                    method=method,
                    quality_score=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'error': f'Unsupported method: {method}'}
                )
                return None, report
            
            self.stats['method_usage'][method.value] += 1
            
            # Calculate scale factors
            scale_factors = self._calculate_scale_factors(downsampled_data.shape, target_shape)
            
            # Check if scaling is reasonable
            if not self._validate_scale_factors(scale_factors):
                report = ReconstructionReport(
                    result="excessive_scaling",
                    original_shape=downsampled_data.shape,
                    reconstructed_shape=target_shape,
                    scale_factor=scale_factors,
                    method=method,
                    quality_score=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'error': f'Excessive scale factors: {scale_factors}'}
                )
                return None, report
            
            # Perform reconstruction
            reconstruction_func = self.supported_methods[method]
            reconstructed_data = reconstruction_func(downsampled_data, target_shape, metadata)
            
            if reconstructed_data is None:
                report = ReconstructionReport(
                    result="reconstruction_failed",
                    original_shape=downsampled_data.shape,
                    reconstructed_shape=target_shape,
                    scale_factor=scale_factors,
                    method=method,
                    quality_score=0.0,
                    processing_time_ms=(time.time() - start_time) * 1000,
                    metadata={'method': method.value}
                )
                return None, report
            
            # Calculate quality score
            quality_score = 0.0
            if self.config['enable_quality_assessment']:
                quality_score = self._assess_reconstruction_quality(
                    downsampled_data, reconstructed_data, method, metadata
                )
            
            # Update statistics
            self.stats['successful_reconstructions'] += 1
            self._update_average_quality(quality_score)
            
            processing_time = (time.time() - start_time) * 1000
            
            self.logger.debug(f"Downsampling reversal successful: {method.value}, "
                            f"scale: {scale_factors}, "
                            f"quality: {quality_score:.3f}, "
                            f"time: {processing_time:.2f}ms")
            
            report = ReconstructionReport(
                result="success",
                original_shape=downsampled_data.shape,
                reconstructed_shape=reconstructed_data.shape,
                scale_factor=scale_factors,
                method=method,
                quality_score=quality_score,
                processing_time_ms=processing_time,
                metadata={
                    'method': method.value,
                    'input_dtype': str(downsampled_data.dtype),
                    'output_dtype': str(reconstructed_data.dtype),
                    'quality_assessment': self.config['enable_quality_assessment']
                }
            )
            
            return reconstructed_data, report
            
        except Exception as e:
            self.stats['failed_reconstructions'] += 1
            self.logger.error(f"Downsampling reversal failed: {e}")
            
            report = ReconstructionReport(
                result="error",
                original_shape=downsampled_data.shape if 'downsampled_data' in locals() else (0,),
                reconstructed_shape=target_shape,
                scale_factor=(1.0, 1.0),
                method=method or self.config['default_method'],
                quality_score=0.0,
                processing_time_ms=(time.time() - start_time) * 1000,
                metadata={'error': str(e)}
            )
            
            return None, report
    
    def _validate_inputs(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                        metadata: Dict) -> Tuple[bool, str]:
        """Validate input data and target shape"""
        if data.size == 0:
            return False, "Empty input data"
        
        if len(target_shape) != len(data.shape):
            return False, f"Shape dimension mismatch: {len(data.shape)} vs {len(target_shape)}"
        
        # Check if target shape is larger than input shape
        for i, (input_dim, target_dim) in enumerate(zip(data.shape, target_shape)):
            if target_dim < input_dim:
                return False, f"Target dimension {i} is smaller than input: {target_dim} < {input_dim}"
        
        # Check total points limit
        total_points = np.prod(target_shape)
        if total_points > self.config['max_reconstructed_points']:
            return False, f"Too many points to reconstruct: {total_points}"
        
        return True, "Valid"
    
    def _select_optimal_method(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                              metadata: Dict) -> ReconstructionMethod:
        """Select optimal reconstruction method based on data characteristics"""
        scale_factors = self._calculate_scale_factors(data.shape, target_shape)
        max_scale = max(scale_factors)
        
        # For large scale factors, use more sophisticated methods
        if max_scale > 4.0 and self.sr_model is not None:
            return ReconstructionMethod.SUPER_RESOLUTION
        elif max_scale > 2.0:
            return ReconstructionMethod.LANCZOS
        elif data.dtype in [np.float32, np.float64]:
            return ReconstructionMethod.BICUBIC
        else:
            return ReconstructionMethod.BILINEAR
    
    def _calculate_scale_factors(self, input_shape: Tuple[int, ...], 
                                target_shape: Tuple[int, ...]) -> Tuple[float, ...]:
        """Calculate scale factors for each dimension"""
        scale_factors = []
        for inp_dim, target_dim in zip(input_shape, target_shape):
            if inp_dim == 0:
                scale_factors.append(1.0)
            else:
                scale_factors.append(target_dim / inp_dim)
        return tuple(scale_factors)
    
    def _validate_scale_factors(self, scale_factors: Tuple[float, ...]) -> bool:
        """Validate that scale factors are reasonable"""
        for factor in scale_factors:
            if factor < self.config['min_scale_factor']:
                return False
            if factor > self.config['max_scale_factor']:
                return False
        return True
    
    def _reconstruct_nearest(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                            metadata: Dict) -> Optional[np.ndarray]:
        """Nearest neighbor reconstruction"""
        try:
            scale_factors = self._calculate_scale_factors(data.shape, target_shape)
            reconstructed = zoom(data, scale_factors, order=0)
            return reconstructed
        except Exception as e:
            self.logger.error(f"Nearest neighbor reconstruction failed: {e}")
            return None
    
    def _reconstruct_bilinear(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                             metadata: Dict) -> Optional[np.ndarray]:
        """Bilinear interpolation reconstruction"""
        try:
            scale_factors = self._calculate_scale_factors(data.shape, target_shape)
            reconstructed = zoom(data, scale_factors, order=1)
            return reconstructed
        except Exception as e:
            self.logger.error(f"Bilinear reconstruction failed: {e}")
            return None
    
    def _reconstruct_bicubic(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                            metadata: Dict) -> Optional[np.ndarray]:
        """Bicubic interpolation reconstruction"""
        try:
            scale_factors = self._calculate_scale_factors(data.shape, target_shape)
            reconstructed = zoom(data, scale_factors, order=3)
            return reconstructed
        except Exception as e:
            self.logger.error(f"Bicubic reconstruction failed: {e}")
            return None
    
    def _reconstruct_lanczos(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                            metadata: Dict) -> Optional[np.ndarray]:
        """Lanczos interpolation reconstruction"""
        try:
            if len(data.shape) == 2:
                # For 2D data, use OpenCV for Lanczos
                reconstructed = cv2.resize(data, (target_shape[1], target_shape[0]), 
                                         interpolation=cv2.INTER_LANCZOS4)
            else:
                # For other dimensions, use zoom with high order
                scale_factors = self._calculate_scale_factors(data.shape, target_shape)
                reconstructed = zoom(data, scale_factors, order=5)  # High order approximates Lanczos
            return reconstructed
        except Exception as e:
            self.logger.error(f"Lanczos reconstruction failed: {e}")
            return None
    
    def _reconstruct_super_resolution(self, data: np.ndarray, target_shape: Tuple[int, ...], 
                                     metadata: Dict) -> Optional[np.ndarray]:
        """Super-resolution based reconstruction"""
        try:
            if self.sr_model is None:
                self.logger.warning("Super-resolution model not available, falling back to bicubic")
                return self._reconstruct_bicubic(data, target_shape, metadata)
            
            # This would use a pre-trained super-resolution model
            # For now, we'll implement a simple example using OpenCV
            if len(data.shape) == 2:
                # Simple super-resolution using iterative back projection
                scale_factors = self._calculate_scale_factors(data.shape, target_shape)
                
                # Start with bicubic upscaling
                initial_upscale = self._reconstruct_bicubic(data, target_shape, metadata)
                if initial_upscale is None:
                    return None
                
                # Apply simple sharpening to enhance details
                kernel = np.array([[-1, -1, -1],
                                   [-1,  9, -1],
                                   [-1, -1, -1]])
                sharpened = cv2.filter2D(initial_upscale, -1, kernel)
                
                return sharpened
            else:
                # For non-2D data, fall back to Lanczos
                return self._reconstruct_lanczos(data, target_shape, metadata)
                
        except Exception as e:
            self.logger.error(f"Super-resolution reconstruction failed: {e}")
            return None
    
    def _load_super_resolution_model(self):
        """Load super-resolution model if available"""
        try:
            # This would load a pre-trained model (ESPCN, FSRCNN, etc.)
            # For now, we'll just set a placeholder
            if self.config['super_resolution_model_path']:
                self.logger.info("Super-resolution model loading would happen here")
                # Example: self.sr_model = cv2.dnn.readNetFromTensorflow('model.pb')
            else:
                self.logger.info("No super-resolution model path configured")
        except Exception as e:
            self.logger.warning(f"Failed to load super-resolution model: {e}")
    
    def _assess_reconstruction_quality(self, original: np.ndarray, reconstructed: np.ndarray,
                                      method: ReconstructionMethod, metadata: Dict) -> float:
        """Assess the quality of reconstruction"""
        try:
            # Downsample the reconstruction to original size for comparison
            original_shape = original.shape
            scale_factors = self._calculate_scale_factors(reconstructed.shape, original_shape)
            
            if not self._validate_scale_factors(scale_factors):
                return 0.5  # Default quality if we can't verify
            
            # Downsample reconstructed data
            downsampled_recon = zoom(reconstructed, [1/s for s in scale_factors], order=1)
            
            # Ensure shapes match
            min_shape = [min(orig, recon) for orig, recon in zip(original_shape, downsampled_recon.shape)]
            original_cropped = original[
                tuple(slice(0, dim) for dim in min_shape)
            ]
            recon_cropped = downsampled_recon[
                tuple(slice(0, dim) for dim in min_shape)
            ]
            
            # Calculate metrics
            mse = np.mean((original_cropped - recon_cropped) ** 2)
            max_val = np.max(original_cropped) - np.min(original_cropped)
            if max_val == 0:
                psnr = 100  # Perfect if no variation
            else:
                psnr = 20 * np.log10(max_val / np.sqrt(mse)) if mse > 0 else 100
            
            # Normalize PSNR to 0-1 scale (assuming PSNR > 20 is good)
            quality = min(psnr / 50.0, 1.0)
            
            return quality
            
        except Exception as e:
            self.logger.warning(f"Quality assessment failed: {e}")
            return 0.5  # Default quality score
    
    def reconstruct_point_cloud(self, points: np.ndarray, target_point_count: int,
                               method: ReconstructionMethod = None) -> Tuple[Optional[np.ndarray], ReconstructionReport]:
        """
        Specialized reconstruction for 3D point clouds
        
        Args:
            points: 3D point cloud (N, 3) or (N, 4) with intensity
            target_point_count: Desired number of points
            method: Reconstruction method
            
        Returns:
            Tuple of (reconstructed_points, report)
        """
        if points.ndim != 2 or points.shape[1] not in [3, 4]:
            report = ReconstructionReport(
                result="invalid_point_cloud",
                original_shape=points.shape,
                reconstructed_shape=(target_point_count, points.shape[1]),
                scale_factor=(1.0, 1.0),
                method=method or self.config['default_method'],
                quality_score=0.0,
                processing_time_ms=0.0,
                metadata={'error': 'Invalid point cloud format'}
            )
            return None, report
        
        current_count = points.shape[0]
        if target_point_count <= current_count:
            # No upsampling needed
            report = ReconstructionReport(
                result="no_upsampling_needed",
                original_shape=points.shape,
                reconstructed_shape=points.shape,
                scale_factor=(1.0, 1.0),
                method=ReconstructionMethod.NEAREST_NEIGHBOR,
                quality_score=1.0,
                processing_time_ms=0.0,
                metadata={'note': 'Target point count <= current count'}
            )
            return points, report
        
        try:
            # For point clouds, we need different approach
            scale_factor = target_point_count / current_count
            
            if method is None:
                if scale_factor > 4.0:
                    method = ReconstructionMethod.SUPER_RESOLUTION
                else:
                    method = ReconstructionMethod.BICUBIC
            
            # Simple point cloud upsampling using interpolation
            if method in [ReconstructionMethod.BILINEAR, ReconstructionMethod.BICUBIC]:
                reconstructed = self._upsample_point_cloud_interpolation(points, target_point_count)
            else:
                # Fall back to interpolation for other methods
                reconstructed = self._upsample_point_cloud_interpolation(points, target_point_count)
            
            if reconstructed is None:
                report = ReconstructionReport(
                    result="point_cloud_reconstruction_failed",
                    original_shape=points.shape,
                    reconstructed_shape=(target_point_count, points.shape[1]),
                    scale_factor=(scale_factor, 1.0),
                    method=method,
                    quality_score=0.0,
                    processing_time_ms=0.0,
                    metadata={'error': 'Point cloud reconstruction failed'}
                )
                return None, report
            
            # Create report
            report = ReconstructionReport(
                result="success",
                original_shape=points.shape,
                reconstructed_shape=reconstructed.shape,
                scale_factor=(scale_factor, 1.0),
                method=method,
                quality_score=0.8,  # Estimate for point clouds
                processing_time_ms=0.0,
                metadata={
                    'point_cloud_upsampling': True,
                    'original_point_count': current_count,
                    'reconstructed_point_count': reconstructed.shape[0]
                }
            )
            
            return reconstructed, report
            
        except Exception as e:
            self.logger.error(f"Point cloud reconstruction failed: {e}")
            report = ReconstructionReport(
                result="error",
                original_shape=points.shape,
                reconstructed_shape=(target_point_count, points.shape[1]),
                scale_factor=(1.0, 1.0),
                method=method or self.config['default_method'],
                quality_score=0.0,
                processing_time_ms=0.0,
                metadata={'error': str(e)}
            )
            return None, report
    
    def _upsample_point_cloud_interpolation(self, points: np.ndarray, target_count: int) -> Optional[np.ndarray]:
        """Upsample point cloud using interpolation"""
        try:
            current_count = points.shape[0]
            if current_count >= target_count:
                return points
            
            # Create parameterization (simple linear)
            t_original = np.linspace(0, 1, current_count)
            t_target = np.linspace(0, 1, target_count)
            
            # Interpolate each dimension
            interpolated_points = []
            for dim in range(points.shape[1]):
                interpolator = interpolate.interp1d(t_original, points[:, dim], 
                                                  kind='cubic', 
                                                  fill_value='extrapolate')
                interpolated_dim = interpolator(t_target)
                interpolated_points.append(interpolated_dim)
            
            reconstructed = np.column_stack(interpolated_points)
            return reconstructed
            
        except Exception as e:
            self.logger.error(f"Point cloud interpolation failed: {e}")
            return None
    
    def _update_average_quality(self, quality: float):
        """Update average quality score"""
        current_avg = self.stats['average_quality_score']
        total_success = self.stats['successful_reconstructions']
        
        if total_success == 1:
            self.stats['average_quality_score'] = quality
        else:
            self.stats['average_quality_score'] = (
                current_avg * (total_success - 1) + quality
            ) / total_success
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get reverser statistics"""
        success_rate = (
            self.stats['successful_reconstructions'] / self.stats['total_reconstructions'] 
            if self.stats['total_reconstructions'] > 0 else 0.0
        )
        
        return {
            **self.stats,
            'success_rate': success_rate,
            'config': {
                'max_scale_factor': self.config['max_scale_factor'],
                'default_method': self.config['default_method'].value
            }
        }

# Utility functions
def reconstruct_resolution(data: np.ndarray, target_shape: Tuple[int, ...]) -> Optional[np.ndarray]:
    """Simple resolution reconstruction"""
    reverser = DownsamplingReverser()
    result, _ = reverser.reconstruct(data, target_shape)
    return result

def upsample_point_cloud(points: np.ndarray, target_count: int) -> Optional[np.ndarray]:
    """Simple point cloud upsampling"""
    reverser = DownsamplingReverser()
    result, _ = reverser.reconstruct_point_cloud(points, target_count)
    return result

# Example usage
if __name__ == "__main__":
    # Test with sample 2D data
    original_data = np.random.rand(32, 32).astype(np.float32)
    target_shape = (128, 128)
    
    reverser = DownsamplingReverser()
    result, report = reverser.reconstruct(original_data, target_shape)
    
    print("Downsampling Reversal Test:")
    print(f"Input shape: {original_data.shape}")
    print(f"Output shape: {result.shape if result is not None else 'None'}")
    print(f"Method: {report.method.value}")
    print(f"Quality Score: {report.quality_score:.3f}")
    print(f"Scale Factors: {report.scale_factor}")
    
    print("\nReverser Statistics:")
    stats = reverser.get_statistics()
    for key, value in stats.items():
        if key != 'config' and key != 'method_usage':
            print(f"  {key}: {value}")