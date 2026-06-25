import { Wifi, WifiOff, Loader2, AlertCircle, Settings, Zap } from 'lucide-react'
import { motion } from 'framer-motion'
import { useStore } from '@/store/store'
import { cn } from '@/utils/helpers'

const STATUS_CONFIG = {
  connected:    { icon: Wifi,     color: 'text-success',  bg: 'bg-success/10',  label: 'Connected' },
  connecting:   { icon: Loader2,  color: 'text-warning',  bg: 'bg-warning/10',  label: 'Connecting…' },
  disconnected: { icon: WifiOff,  color: 'text-muted',    bg: 'bg-muted/10',    label: 'Disconnected' },
  error:        { icon: AlertCircle, color: 'text-danger', bg: 'bg-danger/10',  label: 'Error' },
}

export function Header() {
  const { connectionStatus, robotOnline } = useStore()
  const cfg = STATUS_CONFIG[connectionStatus]
  const Icon = cfg.icon

  return (
    <header className="h-14 flex items-center justify-between px-6 border-b border-border bg-surface/80 backdrop-blur-sm sticky top-0 z-50">
      {/* Brand */}
      <div className="flex items-center gap-3">
        <div className="relative">
          <div className="w-8 h-8 rounded-lg bg-accent/20 flex items-center justify-center border border-accent/40">
            <Zap className="w-4 h-4 text-accent" />
          </div>
          {robotOnline && (
            <span className="absolute -top-0.5 -right-0.5 w-2 h-2 rounded-full bg-success animate-pulse-slow" />
          )}
        </div>
        <div>
          <p className="text-text-primary font-semibold text-sm leading-none">Franka Panda</p>
          <p className="text-text-secondary text-xs mt-0.5">Control Dashboard</p>
        </div>
      </div>

      {/* Center status */}
      <div className="hidden md:flex items-center gap-6">
        <StatusPill label="ROS2" online={connectionStatus === 'connected'} />
        <StatusPill label="Robot" online={robotOnline} />
        <StatusPill label="MoveIt2" online={false} dim />
      </div>

      {/* Right: connection badge + settings */}
      <div className="flex items-center gap-3">
        <motion.div
          className={cn('flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-medium', cfg.bg, cfg.color)}
          animate={connectionStatus === 'connecting' ? { opacity: [1, 0.5, 1] } : {}}
          transition={{ repeat: Infinity, duration: 1.2 }}
        >
          <Icon className={cn('w-3.5 h-3.5', connectionStatus === 'connecting' && 'animate-spin')} />
          {cfg.label}
        </motion.div>
        <button className="w-8 h-8 rounded-lg border border-border hover:border-accent/50 hover:bg-accent/5 flex items-center justify-center transition-colors">
          <Settings className="w-4 h-4 text-text-secondary" />
        </button>
      </div>
    </header>
  )
}

function StatusPill({ label, online, dim }: { label: string; online: boolean; dim?: boolean }) {
  return (
    <div className={cn('flex items-center gap-1.5 text-xs', dim && 'opacity-40')}>
      <span className={cn('w-1.5 h-1.5 rounded-full', online ? 'bg-success' : 'bg-muted')} />
      <span className="text-text-secondary">{label}</span>
    </div>
  )
}
