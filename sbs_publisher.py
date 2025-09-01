# sbs_publisher.py
# -*- coding: utf-8 -*-
"""
Enhanced SBS/BaseStation publisher for QGC with auto-restart capability
- Start a TCP server (default 0.0.0.0:30003)
- QGC connects with "Connect to ADSB SBS server"
- Call publish(lat, lon, alt_amsl_m) at ~1 Hz to draw a traffic marker
- Auto-restart server on connection failures
- Thread-safe operations with proper error handling
"""
import socket
import threading
import time
import logging
from typing import Optional, Tuple

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SBSPublisher:
    def __init__(self, host="0.0.0.0", port=30003, hexid="ABCDEF", callsign="TARGET"):
        self.host = host
        self.port = port
        self.hexid = (hexid or "ABCDEF")[:6]  # SBS format limit
        self.callsign = (callsign or "TARGET")[:8]  # SBS format limit
        
        # Server state
        self._srv: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._stop = False
        self._accept_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        
        # Thread safety
        self._conn_lock = threading.RLock()
        self._server_lock = threading.RLock()
        
        # Auto-restart parameters
        self._auto_restart_enabled = True
        self._restart_delay = 2.0  # seconds
        self._max_restart_attempts = 10
        self._restart_count = 0
        self._last_successful_publish = time.time()
        self._connection_timeout = 30.0  # seconds
        
        # Statistics
        self._total_publishes = 0
        self._failed_publishes = 0
        self._client_connections = 0

    def start(self):
        """Start SBS server with auto-restart capability"""
        with self._server_lock:
            if self._srv:
                logger.info("SBS server already running")
                return
                
            self._stop = False
            self._restart_count = 0
            self._start_server()

    def _start_server(self):
        """Internal server startup with comprehensive error handling"""
        try:
            logger.info(f"Starting SBS server on {self.host}:{self.port}")
            
            # Create and configure server socket
            self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._srv.settimeout(1.0)  # Allow periodic checks for stop flag
            
            # Bind and listen
            self._srv.bind((self.host, self.port))
            self._srv.listen(1)
            
            # Start worker threads
            self._accept_thread = threading.Thread(
                target=self._accept_loop, 
                name="SBS-Accept",
                daemon=True
            )
            self._accept_thread.start()
            
            # Start monitoring thread for auto-restart
            if self._auto_restart_enabled:
                self._monitor_thread = threading.Thread(
                    target=self._monitor_loop,
                    name="SBS-Monitor", 
                    daemon=True
                )
                self._monitor_thread.start()
            
            logger.info(f"SBS server started successfully on {self.host}:{self.port}")
            
        except Exception as e:
            logger.error(f"Failed to start SBS server: {e}")
            self._cleanup_server()
            if self._auto_restart_enabled:
                self._schedule_restart()
            else:
                raise

    def _accept_loop(self):
        """Accept client connections with error recovery"""
        logger.info("SBS accept loop started")
        
        while not self._stop:
            try:
                if not self._srv:
                    logger.warning("Server socket is None in accept loop")
                    break
                    
                try:
                    # Accept new connection with timeout
                    conn, addr = self._srv.accept()
                    logger.info(f"SBS client connected from {addr}")
                    self._client_connections += 1
                    
                    # Replace previous connection atomically
                    with self._conn_lock:
                        # Close old connection if exists
                        if self._conn:
                            try:
                                self._conn.close()
                                logger.debug("Closed previous client connection")
                            except Exception:
                                pass
                        
                        # Set new connection
                        self._conn = conn
                        self._last_successful_publish = time.time()
                        
                except socket.timeout:
                    # Normal timeout, check stop flag and continue
                    continue
                    
                except OSError as e:
                    if not self._stop:
                        logger.warning(f"Accept error: {e}")
                        if self._auto_restart_enabled:
                            self._schedule_restart()
                    break
                    
            except Exception as e:
                if not self._stop:
                    logger.error(f"Unexpected error in accept loop: {e}")
                    if self._auto_restart_enabled:
                        self._schedule_restart()
                break
        
        logger.info("SBS accept loop stopped")

    def _monitor_loop(self):
        """Monitor server health and restart if needed"""
        logger.info("SBS monitor loop started")
        
        while not self._stop:
            try:
                time.sleep(5.0)  # Check every 5 seconds
                
                current_time = time.time()
                time_since_publish = current_time - self._last_successful_publish
                
                # Check for stale connections
                with self._conn_lock:
                    if (self._conn and 
                        time_since_publish > self._connection_timeout):
                        logger.warning(f"Connection appears stale ({time_since_publish:.1f}s since last publish)")
                        try:
                            self._conn.close()
                        except Exception:
                            pass
                        finally:
                            self._conn = None
                
                # Check server socket health
                with self._server_lock:
                    if not self._srv and not self._stop:
                        logger.warning("Server socket is None, scheduling restart")
                        self._schedule_restart()
                        
            except Exception as e:
                logger.error(f"Monitor loop error: {e}")
        
        logger.info("SBS monitor loop stopped")

    def _schedule_restart(self):
        """Schedule server restart in separate thread"""
        if self._stop or self._restart_count >= self._max_restart_attempts:
            if self._restart_count >= self._max_restart_attempts:
                logger.error(f"Max restart attempts ({self._max_restart_attempts}) reached, giving up")
            return
            
        # Start restart in separate thread to avoid blocking
        restart_thread = threading.Thread(
            target=self._restart_server,
            name="SBS-Restart",
            daemon=True
        )
        restart_thread.start()

    def _restart_server(self):
        """Restart server with exponential backoff"""
        with self._server_lock:
            if self._stop:
                return
                
            self._restart_count += 1
            logger.info(f"Restarting SBS server (attempt {self._restart_count}/{self._max_restart_attempts})")
            
            # Cleanup current server
            self._cleanup_server()
            
            # Exponential backoff delay
            delay = min(self._restart_delay * (2 ** (self._restart_count - 1)), 30.0)
            logger.info(f"Waiting {delay:.1f}s before restart...")
            time.sleep(delay)
            
            if self._stop:
                return
                
            # Attempt restart
            try:
                self._start_server()
                self._restart_count = 0  # Reset counter on successful start
                logger.info("SBS server restart successful")
            except Exception as e:
                logger.error(f"Server restart failed: {e}")
                # Will be retried by monitor loop

    def publish(self, lat_deg: float, lon_deg: float, alt_amsl_m: float) -> bool:
        """
        Publish position data to connected clients
        Returns True on success, False on failure
        Thread-safe operation with input validation
        """
        with self._conn_lock:
            if not self._conn:
                return False
                
            try:
                # Input validation and sanitization
                lat_deg = float(lat_deg)
                lon_deg = float(lon_deg)
                alt_amsl_m = float(alt_amsl_m)
                
                # Clamp to reasonable ranges
                lat_deg = max(-90.0, min(90.0, lat_deg))
                lon_deg = max(-180.0, min(180.0, lon_deg))
                alt_amsl_m = max(-1000.0, min(50000.0, alt_amsl_m))
                
                # Format timestamp and altitude
                ts = time.strftime("%Y/%m/%d,%H:%M:%S.000")
                alt_ft = int(alt_amsl_m * 3.28084)  # BaseStation requires feet
                
                # MSG,1 — Aircraft identification
                line1 = (f"MSG,1,1,1,{self.hexid},1,{ts},{ts},"
                        f"{self.callsign},,,,,,,,,,0\r\n")
                
                # MSG,3 — Airborne position (lat/lon/alt)  
                line3 = (f"MSG,3,1,1,{self.hexid},1,{ts},{ts},"
                        f"{self.callsign},{alt_ft},0,0,{lat_deg:.6f},"
                        f"{lon_deg:.6f},0,1200,0,0,0,0\r\n")
                
                # Send data atomically
                data = (line1 + line3).encode("ascii")
                self._conn.sendall(data)
                
                # Update statistics
                self._total_publishes += 1
                self._last_successful_publish = time.time()
                
                return True
                
            except (OSError, socket.error) as e:
                logger.debug(f"Publish failed (connection error): {e}")
                self._failed_publishes += 1
                
                # Close failed connection
                try:
                    self._conn.close()
                except Exception:
                    pass
                finally:
                    self._conn = None
                
                return False
                
            except (ValueError, TypeError) as e:
                logger.error(f"Invalid publish data: lat={lat_deg}, lon={lon_deg}, alt={alt_amsl_m} - {e}")
                self._failed_publishes += 1
                return False
                
            except Exception as e:
                logger.error(f"Unexpected publish error: {e}")
                self._failed_publishes += 1
                return False

    def stop(self):
        """Stop server and cleanup all resources"""
        logger.info("Stopping SBS server...")
        self._stop = True
        
        # Cleanup server resources
        self._cleanup_server()
        
        # Wait for threads to finish gracefully
        for thread_name, thread in [
            ("accept", self._accept_thread),
            ("monitor", self._monitor_thread)
        ]:
            if thread and thread.is_alive():
                logger.debug(f"Waiting for {thread_name} thread to finish...")
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"{thread_name} thread did not finish gracefully")
        
        logger.info("SBS server stopped")

    def _cleanup_server(self):
        """Cleanup all server resources"""
        # Close client connection
        with self._conn_lock:
            if self._conn:
                try:
                    self._conn.close()
                    logger.debug("Client connection closed")
                except Exception:
                    pass
                finally:
                    self._conn = None
        
        # Close server socket
        with self._server_lock:
            if self._srv:
                try:
                    self._srv.close()
                    logger.debug("Server socket closed")
                except Exception:
                    pass
                finally:
                    self._srv = None

    # Properties for status monitoring
    @property
    def is_running(self) -> bool:
        """Check if server is running"""
        with self._server_lock:
            return self._srv is not None and not self._stop

    @property
    def has_client(self) -> bool:
        """Check if client is connected"""
        with self._conn_lock:
            return self._conn is not None

    @property
    def auto_restart_enabled(self) -> bool:
        """Check if auto-restart is enabled"""
        return self._auto_restart_enabled

    @auto_restart_enabled.setter
    def auto_restart_enabled(self, enabled: bool):
        """Enable/disable auto-restart feature"""
        self._auto_restart_enabled = enabled
        logger.info(f"Auto-restart {'enabled' if enabled else 'disabled'}")

    def get_statistics(self) -> dict:
        """Get server statistics and status"""
        return {
            'running': self.is_running,
            'has_client': self.has_client,
            'total_publishes': self._total_publishes,
            'failed_publishes': self._failed_publishes,
            'success_rate': (self._total_publishes / max(1, self._total_publishes + self._failed_publishes)) * 100,
            'client_connections': self._client_connections,
            'restart_count': self._restart_count,
            'last_successful_publish': self._last_successful_publish,
            'auto_restart_enabled': self._auto_restart_enabled
        }

    def reset_statistics(self):
        """Reset all statistics counters"""
        self._total_publishes = 0
        self._failed_publishes = 0
        self._client_connections = 0
        self._restart_count = 0
        logger.info("Statistics reset")

# Example usage and testing
if __name__ == "__main__":
    import random
    
    # Create and start SBS publisher
    sbs = SBSPublisher(port=30003, hexid="TEST01", callsign="TESTBIRD")
    
    try:
        sbs.start()
        print("SBS server started. Connect QGC to localhost:30003")
        print("Press Ctrl+C to stop...")
        
        # Simulate publishing data
        lat, lon, alt = 47.3977508, 8.5455938, 500.0
        
        while True:
            # Simulate small movements
            test_lat = lat + random.uniform(-0.001, 0.001)
            test_lon = lon + random.uniform(-0.001, 0.001)
            test_alt = alt + random.uniform(-10, 10)
            
            success = sbs.publish(test_lat, test_lon, test_alt)
            
            # Print status every 10 publishes
            stats = sbs.get_statistics()
            if stats['total_publishes'] % 10 == 0:
                print(f"Published: {stats['total_publishes']}, "
                      f"Success rate: {stats['success_rate']:.1f}%, "
                      f"Connected: {stats['has_client']}")
            
            time.sleep(1.0)  # 1Hz publishing
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sbs.stop()
