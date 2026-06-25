import { useState } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { Play, Square, CheckCircle2, XCircle, Loader2 } from 'lucide-react'
import { useStore } from '@/store/store'
import { cn } from '@/utils/helpers'
import type { ActionGoal } from '@/types/ros.types'

const ACTION_DEFS = [
  { name: 'Pick', server: '/franka/pick', type: 'franka_web_gui/PickAction', description: 'Pick an object from the scene', color: 'accent' },
  { name: 'Place', server: '/franka/place', type: 'franka_web_gui/PlaceAction', description: 'Place object at target location', color: 'success' },
  { name: 'Wave', server: '/franka/wave', type: 'franka_web_gui/WaveAction', description: 'Perform wave gesture', color: 'warning' },
  { name: 'Handshake', server: '/franka/handshake', type: 'franka_web_gui/HandshakeAction', description: 'Execute handshake motion', color: 'accent' },
]

export function ActionsPage() {
  const { actionGoals, upsertActionGoal, cancelActionGoal, addLog } = useStore()
  const [runningId, setRunningId] = useState<string | null>(null)

  const sendAction = async (def: typeof ACTION_DEFS[0]) => {
    const id = `${def.server}-${Date.now()}`
    const goal: ActionGoal = {
      id,
      name: def.name,
      status: 'pending',
      progress: 0,
      feedback: 'Sending goal…',
      startTime: new Date(),
    }
    upsertActionGoal(goal)
    setRunningId(id)
    addLog({ level: 'info', source: def.server, message: `Goal sent: ${def.name}` })

    // Simulate action execution
    let progress = 0
    const tick = setInterval(() => {
      progress += Math.random() * 12
      if (progress >= 100) {
        progress = 100
        clearInterval(tick)
        upsertActionGoal({ ...goal, status: 'succeeded', progress: 100, feedback: 'Action completed successfully' })
        addLog({ level: 'success', source: def.server, message: `${def.name} succeeded` })
        setRunningId(null)
        return
      }
      upsertActionGoal({ ...goal, status: 'executing', progress, feedback: `Executing… ${progress.toFixed(0)}%` })
    }, 300)
  }

  const cancel = (id: string) => {
    cancelActionGoal(id)
    setRunningId(null)
    addLog({ level: 'warn', source: 'Actions', message: `Goal ${id} cancelled` })
  }

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-6">
      <div>
        <h1 className="text-xl font-semibold text-text-primary">Actions</h1>
        <p className="text-sm text-text-secondary mt-0.5">Send ROS2 action goals with live progress tracking</p>
      </div>

      <div className="grid gap-4 sm:grid-cols-2">
        {ACTION_DEFS.map(def => (
          <div key={def.name} className="bg-surface-2 rounded-xl border border-border p-5 space-y-3">
            <div>
              <p className="text-sm font-semibold text-text-primary">{def.name}</p>
              <p className="text-xs text-muted mt-0.5">{def.description}</p>
              <p className="text-xs font-mono text-muted/60 mt-1">{def.server}</p>
            </div>
            <button
              onClick={() => sendAction(def)}
              disabled={runningId !== null}
              className={cn(
                'flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-medium transition-all active:scale-95',
                runningId
                  ? 'bg-surface border border-border text-muted cursor-not-allowed'
                  : 'bg-accent/15 text-accent border border-accent/30 hover:bg-accent/25',
              )}
            >
              <Play className="w-3.5 h-3.5" />
              Send Goal
            </button>
          </div>
        ))}
      </div>

      {/* Active / recent goals */}
      {actionGoals.length > 0 && (
        <div className="space-y-3">
          <p className="text-text-secondary text-xs font-medium uppercase tracking-wider">Goal History</p>
          <AnimatePresence>
            {[...actionGoals].reverse().slice(0, 8).map(goal => (
              <motion.div
                key={goal.id}
                initial={{ opacity: 0, height: 0 }}
                animate={{ opacity: 1, height: 'auto' }}
                exit={{ opacity: 0, height: 0 }}
                className="bg-surface-2 rounded-xl border border-border p-4 space-y-2"
              >
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <GoalStatusIcon status={goal.status} />
                    <span className="text-sm font-medium text-text-primary">{goal.name}</span>
                    <span className={cn('text-xs px-2 py-0.5 rounded-full', goalBadge(goal.status))}>
                      {goal.status}
                    </span>
                  </div>
                  {goal.status === 'executing' && (
                    <button
                      onClick={() => cancel(goal.id)}
                      className="flex items-center gap-1 text-xs text-danger hover:text-danger/80"
                    >
                      <Square className="w-3 h-3" /> Cancel
                    </button>
                  )}
                </div>

                {(goal.status === 'executing' || goal.status === 'pending') && (
                  <div>
                    <div className="flex justify-between text-xs text-muted mb-1">
                      <span>{goal.feedback}</span>
                      <span>{goal.progress.toFixed(0)}%</span>
                    </div>
                    <div className="h-1.5 bg-background rounded-full overflow-hidden">
                      <motion.div
                        className="h-full bg-accent rounded-full"
                        animate={{ width: `${goal.progress}%` }}
                        transition={{ duration: 0.3 }}
                      />
                    </div>
                  </div>
                )}
              </motion.div>
            ))}
          </AnimatePresence>
        </div>
      )}
    </motion.div>
  )
}

function GoalStatusIcon({ status }: { status: ActionGoal['status'] }) {
  if (status === 'succeeded') return <CheckCircle2 className="w-4 h-4 text-success" />
  if (status === 'failed' || status === 'cancelled') return <XCircle className="w-4 h-4 text-danger" />
  if (status === 'executing' || status === 'pending') return <Loader2 className="w-4 h-4 text-accent animate-spin" />
  return <div className="w-4 h-4 rounded-full border border-muted" />
}

function goalBadge(status: ActionGoal['status']) {
  const map: Record<string, string> = {
    succeeded: 'bg-success/10 text-success',
    failed: 'bg-danger/10 text-danger',
    cancelled: 'bg-danger/10 text-danger',
    executing: 'bg-accent/10 text-accent',
    pending: 'bg-warning/10 text-warning',
    idle: 'bg-muted/10 text-muted',
  }
  return map[status] ?? 'bg-muted/10 text-muted'
}
