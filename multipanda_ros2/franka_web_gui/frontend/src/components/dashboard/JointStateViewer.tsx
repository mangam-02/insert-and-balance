import { useStore } from '@/store/store'
import { radToDeg, jointProgress } from '@/utils/helpers'
import { cn } from '@/utils/helpers'

const JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']

export function JointStateViewer() {
  const { jointState } = useStore()
  const positions = jointState?.position ?? Array(7).fill(0)
  const velocities = jointState?.velocity ?? Array(7).fill(0)

  return (
    <div className="bg-surface-2 rounded-xl border border-border p-5 flex flex-col gap-4">
      <div className="flex items-center justify-between">
        <span className="text-text-secondary text-xs font-medium uppercase tracking-wider">Joint States</span>
        <span className="text-xs text-muted font-mono">7-DOF</span>
      </div>

      <div className="space-y-3">
        {JOINT_NAMES.map((name, i) => {
          const pos = positions[i] ?? 0
          const vel = velocities[i] ?? 0
          const pct = jointProgress(pos, i)
          const isMoving = Math.abs(vel) > 0.001

          return (
            <div key={name} className="flex items-center gap-3">
              <span className="text-xs font-mono text-muted w-6">{name}</span>
              <div className="flex-1 h-2 bg-background rounded-full overflow-hidden">
                <div
                  className={cn(
                    'h-full rounded-full transition-all duration-100',
                    isMoving ? 'bg-accent' : 'bg-accent/60',
                  )}
                  style={{ width: `${Math.max(2, pct)}%` }}
                />
              </div>
              <div className="w-20 text-right">
                <span className="text-xs font-mono text-text-primary">{radToDeg(pos).toFixed(1)}°</span>
              </div>
              <div className="w-14 text-right">
                <span className={cn('text-xs font-mono', isMoving ? 'text-warning' : 'text-muted')}>
                  {vel.toFixed(3)}
                </span>
              </div>
            </div>
          )
        })}
      </div>

      <div className="flex justify-between text-xs text-muted pt-1 border-t border-border">
        <span>Position [rad→deg]</span>
        <span>Velocity [rad/s]</span>
      </div>
    </div>
  )
}
