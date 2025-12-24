/**
 * Connection logging utility for debugging purposes
 */

export interface ConnectionLog {
  timestamp: Date;
  eventType: 'connect' | 'disconnect' | 'request' | 'response' | 'error';
  details: string;
}

class ConnectionLogger {
  private logs: ConnectionLog[] = [];
  private enabled: boolean = true;

  /**
   * Enable or disable logging
   */
  setEnabled(enabled: boolean) {
    this.enabled = enabled;
  }

  /**
   * Log a connection event
   */
  log(eventType: ConnectionLog['eventType'], details: string) {
    if (!this.enabled) return;

    const logEntry: ConnectionLog = {
      timestamp: new Date(),
      eventType,
      details,
    };

    this.logs.push(logEntry);
    console.log(`[ConnectionLog - ${eventType.toUpperCase()}]`, details);

    // Keep only the last 100 logs to prevent memory issues
    if (this.logs.length > 100) {
      this.logs = this.logs.slice(-100);
    }
  }

  /**
   * Get all logs
   */
  getLogs(): ConnectionLog[] {
    return [...this.logs]; // Return a copy of the logs
  }

  /**
   * Clear all logs
   */
  clearLogs() {
    this.logs = [];
  }

  /**
   * Export logs as text
   */
  exportLogs(): string {
    return this.logs.map(log => 
      `${log.timestamp.toISOString()} - ${log.eventType.toUpperCase()}: ${log.details}`
    ).join('\n');
  }
}

export default new ConnectionLogger();