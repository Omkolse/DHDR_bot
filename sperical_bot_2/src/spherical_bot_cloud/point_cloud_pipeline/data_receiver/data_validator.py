#!/usr/bin/env python3
"""
Data Validator
Validates incoming point cloud data for integrity and consistency
"""

import logging
import json
import struct
import hashlib
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import numpy as np
from datetime import datetime, timedelta

class ValidationResult(Enum):
    VALID = "valid"
    INVALID = "invalid"
    SUSPICIOUS = "suspicious"
    CORRUPTED = "corrupted"

@dataclass
class ValidationReport:
    """Validation report container"""
    result: ValidationResult
    score: float  # 0.0 to 1.0
    issues: List[str]
    warnings: List[str]
    metadata: Dict[str, Any]

class DataValidator:
    """Validates point cloud data for integrity and quality"""
    
    def __init__(self):
        self.logger = logging.getLogger('data_validator')
        
        # Validation thresholds
        self.thresholds = {
            'max_data_size': 100 * 1024 * 1024,  # 100MB
            'min_data_size': 10,  # 10 bytes
            'max_timestamp_offset': 300,  # 5 minutes
            'min_compression_ratio': 0.01,  # 1%
            'max_compression_ratio': 10.0,  # 10x
            'sequence_gap_tolerance': 10,
            'max_point_count': 1000000,  # 1 million points
        }
        
        # Robot session tracking
        self.robot_sessions: Dict[str, Dict] = {}
        
        # Statistics
        self.validation_stats = {
            'total_validations': 0,
            'valid_data': 0,
            'invalid_data': 0,
            'suspicious_data': 0,
            'corrupted_data': 0,
            'average_validation_score': 0.0
        }
    
    def validate_pointcloud_data(self, data: Dict[str, Any]) -> ValidationReport:
        """
        Validate point cloud data
        
        Args:
            data: Point cloud data dictionary
            
        Returns:
            ValidationReport with validation results
        """
        self.validation_stats['total_validations'] += 1
        
        issues = []
        warnings = []
        validation_score = 1.0
        robot_id = data.get('robot_id', 'unknown')
        
        try:
            # 1. Basic structure validation
            if not self._validate_basic_structure(data, issues):
                return ValidationReport(
                    result=ValidationResult.INVALID,
                    score=0.0,
                    issues=issues,
                    warnings=warnings,
                    metadata={'robot_id': robot_id}
                )
            
            # 2. Data integrity checks
            integrity_score = self._validate_data_integrity(data, issues, warnings)
            validation_score *= integrity_score
            
            # 3. Timestamp validation
            timestamp_score = self._validate_timestamp(data, issues, warnings)
            validation_score *= timestamp_score
            
            # 4. Sequence validation
            sequence_score = self._validate_sequence(data, robot_id, issues, warnings)
            validation_score *= sequence_score
            
            # 5. Compression validation
            compression_score = self._validate_compression(data, issues, warnings)
            validation_score *= compression_score
            
            # 6. Content validation (if data is available)
            content_score = self._validate_content(data, issues, warnings)
            validation_score *= content_score
            
            # 7. Update robot session
            self._update_robot_session(robot_id, data, validation_score)
            
            # Determine final result
            if validation_score >= 0.8:
                result = ValidationResult.VALID
                self.validation_stats['valid_data'] += 1
            elif validation_score >= 0.5:
                result = ValidationResult.SUSPICIOUS
                self.validation_stats['suspicious_data'] += 1
                warnings.append(f"Data is suspicious (score: {validation_score:.2f})")
            else:
                result = ValidationResult.INVALID
                self.validation_stats['invalid_data'] += 1
            
            # Update average score
            self._update_average_score(validation_score)
            
            self.logger.debug(f"Validation for {robot_id}: {result.value} "
                            f"(score: {validation_score:.2f}, issues: {len(issues)})")
            
            return ValidationReport(
                result=result,
                score=validation_score,
                issues=issues,
                warnings=warnings,
                metadata={
                    'robot_id': robot_id,
                    'timestamp': data.get('timestamp'),
                    'sequence_number': data.get('sequence_number'),
                    'data_size': len(str(data.get('data', '')))
                }
            )
            
        except Exception as e:
            self.validation_stats['corrupted_data'] += 1
            self.logger.error(f"Validation error for {robot_id}: {e}")
            
            return ValidationReport(
                result=ValidationResult.CORRUPTED,
                score=0.0,
                issues=[f"Validation error: {str(e)}"],
                warnings=warnings,
                metadata={'robot_id': robot_id, 'error': str(e)}
            )
    
    def _validate_basic_structure(self, data: Dict, issues: List[str]) -> bool:
        """Validate basic data structure"""
        required_fields = ['data', 'timestamp', 'robot_id']
        missing_fields = [field for field in required_fields if field not in data]
        
        if missing_fields:
            issues.append(f"Missing required fields: {', '.join(missing_fields)}")
            return False
        
        # Check data size
        data_size = len(str(data.get('data', '')))
        if data_size > self.thresholds['max_data_size']:
            issues.append(f"Data too large: {data_size} bytes")
            return False
        
        if data_size < self.thresholds['min_data_size']:
            issues.append(f"Data too small: {data_size} bytes")
            return False
        
        return True
    
    def _validate_data_integrity(self, data: Dict, issues: List[str], warnings: List[str]) -> float:
        """Validate data integrity and checksums"""
        score = 1.0
        
        # Check if checksum is provided and validate it
        if 'checksum' in data:
            calculated_checksum = self._calculate_checksum(data['data'])
            if data['checksum'] != calculated_checksum:
                issues.append("Checksum mismatch")
                score *= 0.0
            else:
                score *= 1.0  # Perfect score for matching checksum
        else:
            warnings.append("No checksum provided")
            score *= 0.9  # Slight penalty for missing checksum
        
        # Validate data format
        if not self._validate_data_format(data, issues):
            score *= 0.5
        
        return score
    
    def _validate_timestamp(self, data: Dict, issues: List[str], warnings: List[str]) -> float:
        """Validate timestamp consistency"""
        score = 1.0
        timestamp = data.get('timestamp')
        current_time = datetime.now().timestamp()
        
        if not isinstance(timestamp, (int, float)):
            issues.append("Invalid timestamp format")
            return 0.0
        
        # Check if timestamp is in the future
        if timestamp > current_time + 60:  # 1 minute in future
            issues.append(f"Timestamp is in the future: {timestamp}")
            score *= 0.0
        
        # Check if timestamp is too far in the past
        elif timestamp < current_time - self.thresholds['max_timestamp_offset']:
            warnings.append(f"Timestamp is too old: {timestamp}")
            score *= 0.8
        
        # Check timestamp monotonicity for this robot
        robot_id = data.get('robot_id')
        if robot_id in self.robot_sessions:
            last_timestamp = self.robot_sessions[robot_id].get('last_timestamp')
            if last_timestamp and timestamp < last_timestamp:
                warnings.append("Timestamp went backwards")
                score *= 0.7
        
        return score
    
    def _validate_sequence(self, data: Dict, robot_id: str, issues: List[str], warnings: List[str]) -> float:
        """Validate sequence number consistency"""
        score = 1.0
        sequence_number = data.get('sequence_number')
        
        if sequence_number is None:
            warnings.append("No sequence number provided")
            return 0.9
        
        if not isinstance(sequence_number, int) or sequence_number < 0:
            issues.append("Invalid sequence number")
            return 0.0
        
        # Check sequence continuity
        if robot_id in self.robot_sessions:
            last_sequence = self.robot_sessions[robot_id].get('last_sequence')
            if last_sequence is not None:
                expected_sequence = last_sequence + 1
                if sequence_number != expected_sequence:
                    gap = sequence_number - expected_sequence
                    if gap > 0:  # Missing sequences
                        if gap <= self.thresholds['sequence_gap_tolerance']:
                            warnings.append(f"Sequence gap detected: missing {gap} frames")
                            score *= max(0.9 - (gap * 0.1), 0.5)
                        else:
                            issues.append(f"Large sequence gap: missing {gap} frames")
                            score *= 0.3
                    else:  # Sequence went backwards
                        issues.append("Sequence number went backwards")
                        score *= 0.0
        
        return score
    
    def _validate_compression(self, data: Dict, issues: List[str], warnings: List[str]) -> float:
        """Validate compression parameters"""
        score = 1.0
        
        original_size = data.get('original_size')
        compressed_size = data.get('compressed_size', len(str(data.get('data', ''))))
        
        if original_size and compressed_size:
            compression_ratio = compressed_size / original_size
            
            if compression_ratio < self.thresholds['min_compression_ratio']:
                issues.append(f"Unrealistic compression ratio: {compression_ratio:.4f}")
                score *= 0.0
            elif compression_ratio > self.thresholds['max_compression_ratio']:
                warnings.append(f"High compression ratio: {compression_ratio:.4f}")
                score *= 0.8
            elif compression_ratio > 1.0:
                warnings.append(f"Negative compression: {compression_ratio:.4f}")
                score *= 0.9
        
        return score
    
    def _validate_content(self, data: Dict, issues: List[str], warnings: List[str]) -> float:
        """Validate content-specific rules"""
        score = 1.0
        
        # Check point count if provided
        point_count = data.get('point_count')
        if point_count:
            if point_count > self.thresholds['max_point_count']:
                warnings.append(f"Large point cloud: {point_count} points")
                score *= 0.9
            
            if point_count <= 0:
                issues.append(f"Invalid point count: {point_count}")
                score *= 0.0
        
        # Validate compression type
        compression_type = data.get('compression_type')
        if compression_type and compression_type not in ['rle', 'delta', 'zlib', 'gzip', 'lzma', 'none']:
            warnings.append(f"Unknown compression type: {compression_type}")
            score *= 0.9
        
        return score
    
    def _validate_data_format(self, data: Dict, issues: List[str]) -> bool:
        """Validate data format consistency"""
        data_content = data.get('data')
        
        if isinstance(data_content, str):
            # Check if it's base64 or hex encoded
            if len(data_content) % 4 == 0 and all(c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=' for c in data_content):
                return True
            elif all(c in '0123456789abcdefABCDEF' for c in data_content) and len(data_content) % 2 == 0:
                return True
            else:
                issues.append("Invalid data encoding format")
                return False
        
        elif isinstance(data_content, (bytes, bytearray)):
            return True
        
        else:
            issues.append("Unsupported data type")
            return False
    
    def _calculate_checksum(self, data: Any) -> str:
        """Calculate checksum for data"""
        if isinstance(data, str):
            data_bytes = data.encode('utf-8')
        elif isinstance(data, (bytes, bytearray)):
            data_bytes = data
        else:
            data_bytes = str(data).encode('utf-8')
        
        return hashlib.md5(data_bytes).hexdigest()
    
    def _update_robot_session(self, robot_id: str, data: Dict, score: float):
        """Update robot session information"""
        if robot_id not in self.robot_sessions:
            self.robot_sessions[robot_id] = {
                'first_seen': datetime.now(),
                'last_seen': datetime.now(),
                'total_messages': 0,
                'valid_messages': 0,
                'average_score': 0.0,
                'last_sequence': None,
                'last_timestamp': None
            }
        
        session = self.robot_sessions[robot_id]
        session['last_seen'] = datetime.now()
        session['total_messages'] += 1
        
        if score >= 0.8:
            session['valid_messages'] += 1
        
        # Update average score
        current_avg = session['average_score']
        session['average_score'] = (current_avg * (session['total_messages'] - 1) + score) / session['total_messages']
        
        # Update sequence and timestamp
        sequence_number = data.get('sequence_number')
        if sequence_number is not None:
            session['last_sequence'] = sequence_number
        
        timestamp = data.get('timestamp')
        if timestamp is not None:
            session['last_timestamp'] = timestamp
    
    def _update_average_score(self, score: float):
        """Update global average validation score"""
        current_avg = self.validation_stats['average_validation_score']
        total_validations = self.validation_stats['total_validations']
        
        if total_validations == 1:
            self.validation_stats['average_validation_score'] = score
        else:
            self.validation_stats['average_validation_score'] = (
                current_avg * (total_validations - 1) + score
            ) / total_validations
    
    def get_robot_health_report(self, robot_id: str) -> Dict[str, Any]:
        """Get health report for specific robot"""
        if robot_id not in self.robot_sessions:
            return {'status': 'unknown', 'message': 'Robot not found'}
        
        session = self.robot_sessions[robot_id]
        current_time = datetime.now()
        time_since_last_seen = (current_time - session['last_seen']).total_seconds()
        
        # Determine health status
        if time_since_last_seen > 300:  # 5 minutes
            status = 'offline'
        elif session['average_score'] < 0.5:
            status = 'unhealthy'
        elif session['average_score'] < 0.8:
            status = 'degraded'
        else:
            status = 'healthy'
        
        return {
            'robot_id': robot_id,
            'status': status,
            'first_seen': session['first_seen'].isoformat(),
            'last_seen': session['last_seen'].isoformat(),
            'time_since_last_seen_seconds': time_since_last_seen,
            'total_messages': session['total_messages'],
            'valid_messages': session['valid_messages'],
            'validity_rate': session['valid_messages'] / session['total_messages'] if session['total_messages'] > 0 else 0,
            'average_validation_score': session['average_score'],
            'last_sequence': session['last_sequence'],
            'last_timestamp': session['last_timestamp']
        }
    
    def cleanup_old_sessions(self, max_age_hours: int = 24):
        """Clean up old robot sessions"""
        current_time = datetime.now()
        robots_to_remove = []
        
        for robot_id, session in self.robot_sessions.items():
            age_hours = (current_time - session['last_seen']).total_seconds() / 3600
            if age_hours > max_age_hours:
                robots_to_remove.append(robot_id)
        
        for robot_id in robots_to_remove:
            del self.robot_sessions[robot_id]
            self.logger.info(f"Cleaned up old session for robot: {robot_id}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get validation statistics"""
        active_robots = {
            robot_id: session for robot_id, session in self.robot_sessions.items()
            if (datetime.now() - session['last_seen']).total_seconds() < 300  # 5 minutes
        }
        
        return {
            **self.validation_stats,
            'active_robots': len(active_robots),
            'total_tracked_robots': len(self.robot_sessions),
            'validation_thresholds': self.thresholds
        }

# Example usage
if __name__ == "__main__":
    # Test with sample data
    test_data = {
        'robot_id': 'test_bot_001',
        'timestamp': datetime.now().timestamp(),
        'sequence_number': 1,
        'data': 'A' * 1000,  # Simple test data
        'original_size': 2000,
        'compressed_size': 1000,
        'compression_type': 'rle',
        'point_count': 100
    }
    
    validator = DataValidator()
    report = validator.validate_pointcloud_data(test_data)
    
    print("Validation Report:")
    print(f"  Result: {report.result.value}")
    print(f"  Score: {report.score:.2f}")
    print(f"  Issues: {report.issues}")
    print(f"  Warnings: {report.warnings}")
    
    print("\nValidation Statistics:")
    stats = validator.get_statistics()
    for key, value in stats.items():
        if key != 'validation_thresholds':
            print(f"  {key}: {value}")