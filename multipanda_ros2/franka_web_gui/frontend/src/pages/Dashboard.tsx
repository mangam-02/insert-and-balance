import { motion } from 'framer-motion'
import { RobotStatusCard } from '@/components/dashboard/RobotStatusCard'
import { JointStateViewer } from '@/components/dashboard/JointStateViewer'
import { JointChart } from '@/components/dashboard/JointChart'
import { TopicMonitor } from '@/components/dashboard/TopicMonitor'
import { CameraFeed } from '@/components/dashboard/CameraFeed'
import { EventLog } from '@/components/dashboard/EventLog'
import { useStore } from '@/store/store'
import { AlertTriangle } from 'lucide-react'
import { topicManager } from '@/services/ros/TopicManager'
import { cn } from '@/utils/helpers'

const item = {
  hidden: { opacity: 0, y: 16 },
  show: { opacity: 1, y: 0 },
}

export function Dashboard() {
  const { cameras, connectionStatus } = useStore()
  const color = useStore(s => s.cameras[0])
  const depth = useStore(s => s.cameras[1])

  const publishHello = (payload: string) => {
    topicManager.publish('/hello_world_command', 'std_msgs/String', { data: payload })
    useStore.getState().addLog({ level: 'info', source: '/hello_world_command', message: `Published: ${payload}` })
  }

  return (
    <motion.div
      variants={{ show: { transition: { staggerChildren: 0.06 } } }}
      initial="hidden"
      animate="show"
      className="space-y-5"
    >
      {/* Mock banner */}
      {connectionStatus !== 'connected' && (
        <motion.div variants={item} className="flex items-center gap-2 bg-warning/10 border border-warning/30 rounded-lg px-4 py-2.5 text-warning text-sm">
          <AlertTriangle className="w-4 h-4 flex-shrink-0" />
          <span>ROS Bridge not connected — running in <strong>MOCK mode</strong>. Joint data is simulated.</span>
        </motion.div>
      )}

      {/* Row 1: Status + Joint viewer */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-5">
        <motion.div variants={item}>
          <RobotStatusCard />
        </motion.div>
        <motion.div variants={item} className="lg:col-span-2">
          <JointStateViewer />
        </motion.div>
      </div>

      {/* Row 2: Chart */}
      <motion.div variants={item}>
        <JointChart />
      </motion.div>

      {/* Row 3: Cameras + publish controls */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-5">
        <motion.div variants={item} className="lg:col-span-2 space-y-3">
          <CameraFeed label={color.label} imageDataUrl={color.imageDataUrl} fps={color.fps} active={color.active} />
          <CameraFeed label={depth.label} imageDataUrl={depth.imageDataUrl} fps={depth.fps} active={depth.active} compact />
        </motion.div>

        <motion.div variants={item} className="space-y-4">
          {/* Quick publish */}
          <div className="bg-surface-2 rounded-xl border border-border p-5">
            <p className="text-text-secondary text-xs font-medium uppercase tracking-wider mb-3">Publish /hello_world_command</p>
            <div className="space-y-2">
              {(['start', 'stop', 'reset'] as const).map(cmd => (
                <button
                  key={cmd}
                  onClick={() => publishHello(cmd)}
                  className={cn(
                    'w-full px-4 py-2 rounded-lg text-sm font-medium transition-all active:scale-95',
                    cmd === 'stop'
                      ? 'bg-danger/15 text-danger border border-danger/30 hover:bg-danger/25'
                      : cmd === 'start'
                      ? 'bg-success/15 text-success border border-success/30 hover:bg-success/25'
                      : 'bg-surface border border-border text-text-secondary hover:text-text-primary',
                  )}
                >
                  {cmd.charAt(0).toUpperCase() + cmd.slice(1)}
                </button>
              ))}
            </div>
          </div>

          {/* Topic summary */}
          <TopicMonitor />
        </motion.div>
      </div>

      {/* Row 4: Event log */}
      <motion.div variants={item}>
        <EventLog />
      </motion.div>
    </motion.div>
  )
}
