import { motion } from 'framer-motion'
import { Activity, AlertTriangle, CheckCircle2, XCircle } from 'lucide-react'
import { useStore } from '@/store/store'

export function RobotStatusCard() {
  const { robotOnline, connectionStatus, frankaState } = useStore()

  const statusIcon = robotOnline
    ? <CheckCircle2 className="w-5 h-5 text-success" />
    : <XCircle className="w-5 h-5 text-danger" />

  const collision = frankaState?.cartesian_collision?.some(v => v > 0.5)

  return (
    <div className="bg-surface-2 rounded-xl border border-border p-5 flex flex-col gap-4">
      <div className="flex items-center justify-between">
        <span className="text-text-secondary text-xs font-medium uppercase tracking-wider">Robot Status</span>
        <Activity className="w-4 h-4 text-muted" />
      </div>

      <div className="flex items-center gap-3">
        <motion.div
          animate={robotOnline ? { scale: [1, 1.05, 1] } : {}}
          transition={{ repeat: Infinity, duration: 2 }}
        >
          {statusIcon}
        </motion.div>
        <div>
          <p className={`text-lg font-semibold ${robotOnline ? 'text-success' : 'text-danger'}`}>
            {robotOnline ? 'Online' : 'Offline'}
          </p>
          <p className="text-xs text-text-secondary capitalize">{connectionStatus}</p>
        </div>
      </div>

      <div className="grid grid-cols-2 gap-2 pt-1">
        <MetricBadge label="Mode" value="IDLE" ok />
        <MetricBadge label="Collision" value={collision ? 'DETECTED' : 'Clear'} ok={!collision} />
        <MetricBadge label="E-Stop" value="Inactive" ok />
        <MetricBadge label="Brakes" value="Released" ok />
      </div>
    </div>
  )
}

function MetricBadge({ label, value, ok }: { label: string; value: string; ok: boolean }) {
  return (
    <div className="bg-background rounded-lg px-3 py-2">
      <p className="text-xs text-muted">{label}</p>
      <p className={`text-xs font-semibold mt-0.5 ${ok ? 'text-success' : 'text-danger'}`}>{value}</p>
    </div>
  )
}
