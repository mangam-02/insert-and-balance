import { useState } from 'react'
import { motion } from 'framer-motion'
import { Wrench, CheckCircle2, XCircle, Loader2 } from 'lucide-react'
import { useStore } from '@/store/store'
import { serviceManager } from '@/services/ros/ServiceManager'
import { cn, formatTimestamp } from '@/utils/helpers'

const SERVICE_DEFS = [
  { name: '/robot/start', type: 'std_srvs/Trigger', description: 'Start the robot controller', request: {} },
  { name: '/robot/stop', type: 'std_srvs/Trigger', description: 'Stop the robot controller', request: {} },
  { name: '/robot/home', type: 'std_srvs/Trigger', description: 'Move robot to home position', request: {} },
  { name: '/franka/error_recovery', type: 'franka_msgs/ErrorRecovery', description: 'Recover from error state', request: {} },
]

export function ServicesPage() {
  const { services, updateService, addLog, connectionStatus } = useStore()

  const callService = async (name: string, type: string) => {
    updateService(name, { status: 'calling', lastCallTime: new Date() })
    addLog({ level: 'info', source: 'Services', message: `Calling ${name}…` })

    if (connectionStatus !== 'connected') {
      // Simulate success in mock mode
      await new Promise(r => setTimeout(r, 800))
      updateService(name, { status: 'success', lastResult: '{"success":true,"message":"[MOCK] Service call simulated"}' })
      addLog({ level: 'success', source: name, message: '[MOCK] Service succeeded' })
      return
    }

    try {
      const result = await serviceManager.call(name, type)
      updateService(name, { status: 'success', lastResult: JSON.stringify(result) })
      addLog({ level: 'success', source: name, message: `Service succeeded: ${JSON.stringify(result)}` })
    } catch (err) {
      updateService(name, { status: 'error', lastResult: String(err) })
      addLog({ level: 'error', source: name, message: `Service failed: ${err}` })
    }
  }

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-6">
      <div>
        <h1 className="text-xl font-semibold text-text-primary">Services</h1>
        <p className="text-sm text-text-secondary mt-0.5">Call ROS2 services — mock results when not connected</p>
      </div>

      <div className="grid gap-4 sm:grid-cols-2">
        {SERVICE_DEFS.map(({ name, type, description }) => {
          const svc = services[name]
          const status = svc?.status ?? 'idle'

          return (
            <div key={name} className="bg-surface-2 rounded-xl border border-border p-5 space-y-3">
              <div className="flex items-start justify-between gap-2">
                <div>
                  <p className="text-sm font-mono text-text-primary">{name}</p>
                  <p className="text-xs text-muted mt-0.5">{description}</p>
                </div>
                <StatusIcon status={status} />
              </div>

              <p className="text-xs text-muted font-mono">{type}</p>

              {svc?.lastResult && (
                <div className="bg-background rounded-lg px-3 py-2 text-xs font-mono text-text-secondary break-all">
                  {svc.lastResult}
                </div>
              )}

              <div className="flex items-center justify-between">
                {svc?.lastCallTime && (
                  <span className="text-xs text-muted">{formatTimestamp(svc.lastCallTime)}</span>
                )}
                <button
                  onClick={() => callService(name, type)}
                  disabled={status === 'calling'}
                  className={cn(
                    'ml-auto flex items-center gap-2 px-4 py-2 rounded-lg text-sm font-medium transition-all',
                    status === 'calling'
                      ? 'bg-accent/10 text-accent/60 cursor-not-allowed'
                      : 'bg-accent/15 text-accent border border-accent/30 hover:bg-accent/25 active:scale-95',
                  )}
                >
                  {status === 'calling' ? (
                    <><Loader2 className="w-3.5 h-3.5 animate-spin" /> Calling…</>
                  ) : (
                    <><Wrench className="w-3.5 h-3.5" /> Call</>
                  )}
                </button>
              </div>
            </div>
          )
        })}
      </div>
    </motion.div>
  )
}

function StatusIcon({ status }: { status: string }) {
  if (status === 'success') return <CheckCircle2 className="w-4 h-4 text-success flex-shrink-0" />
  if (status === 'error') return <XCircle className="w-4 h-4 text-danger flex-shrink-0" />
  if (status === 'calling') return <Loader2 className="w-4 h-4 text-accent animate-spin flex-shrink-0" />
  return <div className="w-4 h-4 rounded-full border border-muted flex-shrink-0" />
}
