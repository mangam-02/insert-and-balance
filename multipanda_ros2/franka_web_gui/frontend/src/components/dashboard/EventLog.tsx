import { Terminal, Trash2 } from 'lucide-react'
import { motion, AnimatePresence } from 'framer-motion'
import { useStore } from '@/store/store'
import { cn } from '@/utils/helpers'

const LEVEL_STYLE = {
  info:    'text-accent',
  warn:    'text-warning',
  error:   'text-danger',
  success: 'text-success',
}

export function EventLog() {
  const { logs, clearLogs } = useStore()
  const recent = logs.slice(0, 12)

  return (
    <div className="bg-surface-2 rounded-xl border border-border p-5 flex flex-col gap-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Terminal className="w-4 h-4 text-muted" />
          <span className="text-text-secondary text-xs font-medium uppercase tracking-wider">Event Log</span>
        </div>
        <button onClick={clearLogs} className="text-muted hover:text-danger transition-colors">
          <Trash2 className="w-3.5 h-3.5" />
        </button>
      </div>

      <div className="bg-background rounded-lg p-3 font-mono text-xs space-y-1 max-h-48 overflow-y-auto">
        <AnimatePresence initial={false}>
          {recent.length === 0 ? (
            <p className="text-muted">No events yet…</p>
          ) : (
            recent.map(log => (
              <motion.div
                key={log.id}
                initial={{ opacity: 0, y: -4 }}
                animate={{ opacity: 1, y: 0 }}
                className="flex gap-2 leading-relaxed"
              >
                <span className="text-muted flex-shrink-0">
                  {log.timestamp.toLocaleTimeString('en-US', { hour12: false })}
                </span>
                <span className={cn('flex-shrink-0', LEVEL_STYLE[log.level])}>
                  [{log.level.toUpperCase()}]
                </span>
                <span className="text-text-secondary flex-shrink-0">{log.source}:</span>
                <span className="text-text-primary break-all">{log.message}</span>
              </motion.div>
            ))
          )}
        </AnimatePresence>
      </div>
    </div>
  )
}
