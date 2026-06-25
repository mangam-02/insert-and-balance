import ROSLIB from 'roslib'
import type { ConnectionStatus } from '@/types/ros.types'

type StatusCallback = (status: ConnectionStatus) => void

class RosConnection {
  private ros: ROSLIB.Ros | null = null
  private url: string
  private callbacks: Set<StatusCallback> = new Set()
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  private reconnectDelay = 3000

  constructor(url = 'ws://localhost:9090') {
    this.url = url
  }

  connect(url?: string): void {
    if (url) this.url = url
    this.cleanup()
    this.notify('connecting')

    this.ros = new ROSLIB.Ros({ url: this.url })

    this.ros.on('connection', () => {
      this.reconnectDelay = 3000
      this.notify('connected')
    })

    this.ros.on('error', () => {
      this.notify('error')
      this.scheduleReconnect()
    })

    this.ros.on('close', () => {
      this.notify('disconnected')
      this.scheduleReconnect()
    })
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      this.reconnectDelay = Math.min(this.reconnectDelay * 1.5, 30000)
      this.connect()
    }, this.reconnectDelay)
  }

  private cleanup(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.ros) {
      try { this.ros.close() } catch { /* ignore */ }
      this.ros = null
    }
  }

  disconnect(): void {
    this.cleanup()
    this.notify('disconnected')
  }

  getRos(): ROSLIB.Ros | null {
    return this.ros
  }

  onStatusChange(cb: StatusCallback): () => void {
    this.callbacks.add(cb)
    return () => this.callbacks.delete(cb)
  }

  private notify(status: ConnectionStatus): void {
    this.callbacks.forEach(cb => cb(status))
  }
}

export const rosConnection = new RosConnection()
export default RosConnection
