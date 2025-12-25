import time
import logging
from typing import Dict, Any, Callable, Optional
from functools import wraps
from datetime import datetime
from collections import defaultdict


class PerformanceMonitor:
    """
    Service for monitoring performance metrics and collecting metrics
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.metrics = defaultdict(list)
        self.performance_thresholds = {
            "max_validation_latency": 2.0,  # seconds
            "max_search_latency": 1.0,      # seconds
            "max_embedding_latency": 1.0    # seconds
        }

    def record_metric(self, metric_name: str, value: float, tags: Dict[str, str] = None):
        """
        Record a performance metric
        """
        timestamp = datetime.now().isoformat()
        metric_data = {
            "value": value,
            "timestamp": timestamp,
            "tags": tags or {}
        }

        self.metrics[metric_name].append(metric_data)

        self.logger.info(f"Metric recorded: {metric_name} = {value}")

    def time_function(self, metric_name: str = None, category: str = "general"):
        """
        Decorator to time function execution
        """
        def decorator(func: Callable):
            @wraps(func)
            def wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    return result
                finally:
                    duration = time.time() - start_time
                    name = metric_name or f"{func.__module__}.{func.__name__}.duration"
                    self.record_metric(name, duration)

                    # Check if the duration exceeds thresholds
                    if category in self.performance_thresholds:
                        threshold = self.performance_thresholds[category]
                        if duration > threshold:
                            self.logger.warning(f"Performance threshold exceeded for {name}: {duration:.2f}s > {threshold:.2f}s")

            return wrapper
        return decorator

    def get_metrics_summary(self) -> Dict[str, Dict[str, float]]:
        """
        Get a summary of collected metrics
        """
        summary = {}
        for metric_name, data_points in self.metrics.items():
            values = [point["value"] for point in data_points]
            if values:
                summary[metric_name] = {
                    "count": len(values),
                    "avg": sum(values) / len(values),
                    "min": min(values),
                    "max": max(values),
                    "last": values[-1],
                    "p95": self._calculate_percentile(values, 95),
                    "p99": self._calculate_percentile(values, 99)
                }
        return summary

    def _calculate_percentile(self, values: list, percentile: float) -> float:
        """
        Calculate percentile of values
        """
        if not values:
            return 0.0
        sorted_values = sorted(values)
        index = int(len(sorted_values) * percentile / 100)
        return sorted_values[min(index, len(sorted_values) - 1)]

    def log_performance_metrics(self):
        """
        Log all performance metrics
        """
        summary = self.get_metrics_summary()
        for metric_name, stats in summary.items():
            self.logger.info(f"Performance metric '{metric_name}': {stats}")

    def check_performance_thresholds(self) -> Dict[str, Any]:
        """
        Check if performance metrics exceed defined thresholds
        """
        issues = []
        summary = self.get_metrics_summary()

        for metric_name, stats in summary.items():
            if "validation" in metric_name and stats["avg"] > self.performance_thresholds["max_validation_latency"]:
                issues.append({
                    "metric": metric_name,
                    "type": "latency_exceeded",
                    "value": stats["avg"],
                    "threshold": self.performance_thresholds["max_validation_latency"]
                })

        return {
            "issues_found": len(issues) > 0,
            "issues": issues,
            "summary": summary
        }

    def clear_metrics(self):
        """
        Clear all collected metrics
        """
        self.metrics.clear()
        self.logger.info("Cleared all performance metrics")


# Global performance monitor instance
performance_monitor = PerformanceMonitor()


def monitor_performance(metric_name: str = None, category: str = "general"):
    """
    Convenience decorator to monitor performance
    """
    return performance_monitor.time_function(metric_name, category)