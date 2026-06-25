import { motion } from 'framer-motion'
import {
  Home, Hand, PackageOpen, Grip, Waves, Handshake, CheckCircle2, XCircle, Loader2,
} from 'lucide-react'
import { useStore } from '@/store/store'
import { topicManager } from '@/services/ros/TopicManager'
import { cn } from '@/utils/helpers'
import type { Skill } from '@/types/ros.types'

const ICON_MAP: Record<string, React.ElementType> = {
  Home, Hand, PackageOpen, Grip, Waves, Handshake,
  HandOpen: Hand,
}

export function SkillsPage() {
  const { skills, updateSkill, addLog } = useStore()

  const execute = (skill: Skill) => {
    updateSkill(skill.id, { status: 'running' })
    addLog({ level: 'info', source: 'Skills', message: `Executing skill: ${skill.name}` })

    topicManager.publish(skill.topic, 'std_msgs/String', { data: skill.payload })

    setTimeout(() => {
      updateSkill(skill.id, { status: 'success' })
      addLog({ level: 'success', source: 'Skills', message: `Skill "${skill.name}" completed` })
      setTimeout(() => updateSkill(skill.id, { status: 'idle' }), 2000)
    }, 1500 + Math.random() * 1000)
  }

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-6">
      <div>
        <h1 className="text-xl font-semibold text-text-primary">Skill Library</h1>
        <p className="text-sm text-text-secondary mt-0.5">One-click Franka Panda skills — publish ROS topics to trigger</p>
      </div>

      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
        {skills.map((skill, i) => {
          const Icon = ICON_MAP[skill.icon] ?? Home
          const isRunning = skill.status === 'running'
          const isSuccess = skill.status === 'success'
          const isError = skill.status === 'error'

          return (
            <motion.div
              key={skill.id}
              initial={{ opacity: 0, y: 16 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: i * 0.05 }}
              className={cn(
                'bg-surface-2 rounded-xl border p-5 flex flex-col gap-4 transition-all',
                isRunning ? 'border-accent/40 shadow-lg shadow-accent/10' :
                isSuccess ? 'border-success/40' :
                isError ? 'border-danger/40' :
                'border-border hover:border-border/80',
              )}
            >
              <div className="flex items-start justify-between">
                <div className={cn(
                  'w-10 h-10 rounded-xl flex items-center justify-center',
                  isRunning ? 'bg-accent/20' : 'bg-surface',
                )}>
                  {isRunning
                    ? <Loader2 className="w-5 h-5 text-accent animate-spin" />
                    : isSuccess
                    ? <CheckCircle2 className="w-5 h-5 text-success" />
                    : isError
                    ? <XCircle className="w-5 h-5 text-danger" />
                    : <Icon className="w-5 h-5 text-text-secondary" />
                  }
                </div>
                <span className={cn(
                  'text-xs px-2 py-0.5 rounded-full font-medium',
                  isRunning ? 'bg-accent/15 text-accent' :
                  isSuccess ? 'bg-success/15 text-success' :
                  isError ? 'bg-danger/15 text-danger' :
                  'bg-muted/10 text-muted',
                )}>
                  {skill.status}
                </span>
              </div>

              <div>
                <p className="text-sm font-semibold text-text-primary">{skill.name}</p>
                <p className="text-xs text-text-secondary mt-0.5">{skill.description}</p>
                <p className="text-xs font-mono text-muted mt-1 truncate">{skill.topic} → "{skill.payload}"</p>
              </div>

              <button
                onClick={() => execute(skill)}
                disabled={isRunning}
                className={cn(
                  'mt-auto w-full py-2 rounded-lg text-sm font-medium transition-all active:scale-95',
                  isRunning
                    ? 'bg-accent/10 text-accent/50 cursor-not-allowed'
                    : 'bg-accent/15 text-accent border border-accent/30 hover:bg-accent/25',
                )}
              >
                {isRunning ? 'Running…' : 'Execute'}
              </button>
            </motion.div>
          )
        })}
      </div>
    </motion.div>
  )
}
