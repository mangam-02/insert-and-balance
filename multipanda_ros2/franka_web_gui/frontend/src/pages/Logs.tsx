import { motion } from 'framer-motion'
import { Terminal, Trash2, Download } from 'lucide-react'
import { useStore } from '@/store/store'
import { cn } from '@/utils/helpers'

const LEVEL_STYLE = {
  info:    { text: 'text-accent',   badge: 'bg-accent/10 text-accent' },
  warn:    { text: 'text-warning',  badge: 'bg-warning/10 text-warning' },
  error:   { text: 'text-danger',   badge: 'bg-danger/10 text-danger' },
  success: { text: 'text-success',  badge: 'bg-success/10 text-success' },
}

export function LogsPage() {
  const { logs, clearLogs } = useStore()

  const downloadLogs = () => {
    const text = logs
      .map(l => `[${l.timestamp.toISOString()}] [${l.level.toUpperCase()}] ${l.source}: ${l.message}`)
      .join('\n')
    const blob = new Blob([text], { type: 'text/plain' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url; a.download = `franka-logs-${Date.now()}.txt`; a.click()
    URL.revokeObjectURL(url)
  }

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-4">
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-xl font-semibold text-text-primary">Event Logs</h1>
          <p className="text-sm text-text-secondary mt-0.5">{logs.length} entries</p>
        </div>
        <div className="flex gap-2">
          <button
            onClick={downloadLogs}
            className="flex items-center gap-1.5 px-3 py-1.5 text-xs bg-surface-2 border border-border rounded-lg text-text-secondary hover:text-text-primary transition-colors"
          >
            <Download className="w-3.5 h-3.5" /> Export
          </button>
          <button
            onClick={clearLogs}
            className="flex items-center gap-1.5 px-3 py-1.5 text-xs bg-danger/10 border border-danger/30 rounded-lg text-danger hover:bg-danger/20 transition-colors"
          >
            <Trash2 className="w-3.5 h-3.5" /> Clear
          </button>
        </div>
      </div>

      <div className="bg-surface-2 rounded-xl border border-border overflow-hidden">
        <div className="flex items-center gap-3 px-4 py-2.5 border-b border-border bg-surface">
          <Terminal className="w-4 h-4 text-muted" />
          <span className="text-xs font-mono text-muted">ROS2 / System Events</span>
          <span className="ml-auto text-xs font-mono text-muted">{logs.length} / 500</span>
        </div>

        <div className="font-mono text-xs divide-y divide-border max-h-[calc(100vh-220px)] overflow-y-auto">
          {logs.length === 0 ? (
            <div className="px-4 py-8 text-center text-muted">No log entries yet</div>
          ) : (
            logs.map(log => {
              const style = LEVEL_STYLE[log.level]
              return (
                <div key={log.id} className="flex items-start gap-3 px-4 py-2.5 hover:bg-surface transition-colors">
                  <span className="text-muted flex-shrink-0 w-20">
                    {log.timestamp.toLocaleTimeString('en-US', { hour12: false })}
                  </span>
                  <span className={cn('flex-shrink-0 w-16 text-center px-1.5 py-0.5 rounded text-[10px] font-bold', style.badge)}>
                    {log.level.toUpperCase()}
                  </span>
                  <span className="text-text-secondary flex-shrink-0 w-36 truncate">{log.source}</span>
                  <span className="text-text-primary break-all">{log.message}</span>
                </div>
              )
            })
          )}
        </div>
      </div>
    </motion.div>
  )
}
