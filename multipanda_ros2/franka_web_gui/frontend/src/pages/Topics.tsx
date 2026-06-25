import { useState } from 'react'
import { motion } from 'framer-motion'
import { Radio, Send, ChevronDown, ChevronUp } from 'lucide-react'
import { useStore } from '@/store/store'
import { topicManager } from '@/services/ros/TopicManager'
import { formatTimestamp, cn } from '@/utils/helpers'

export function TopicsPage() {
  const topics = useStore(s => Object.values(s.topics))
  const addLog = useStore(s => s.addLog)
  const [publishValue, setPublishValue] = useState('Hello from Web GUI!')
  const [expanded, setExpanded] = useState<string | null>(null)

  const publish = () => {
    topicManager.publish('/hello_world_command', 'std_msgs/String', { data: publishValue })
    addLog({ level: 'info', source: '/hello_world_command', message: `Published: "${publishValue}"` })
  }

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-6">
      <div>
        <h1 className="text-xl font-semibold text-text-primary">Topic Monitor</h1>
        <p className="text-sm text-text-secondary mt-0.5">Subscribe and publish to ROS2 topics</p>
      </div>

      {/* Publisher */}
      <div className="bg-surface-2 rounded-xl border border-border p-5 space-y-3">
        <p className="text-text-secondary text-xs font-medium uppercase tracking-wider">Publish to /hello_world_command</p>
        <div className="flex gap-3">
          <input
            value={publishValue}
            onChange={e => setPublishValue(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && publish()}
            placeholder="Message payload…"
            className="flex-1 bg-background border border-border rounded-lg px-3 py-2 text-sm text-text-primary placeholder:text-muted focus:outline-none focus:border-accent/60"
          />
          <button
            onClick={publish}
            className="flex items-center gap-2 px-4 py-2 bg-accent/15 text-accent border border-accent/30 rounded-lg text-sm font-medium hover:bg-accent/25 transition-colors"
          >
            <Send className="w-4 h-4" />
            Publish
          </button>
        </div>
        <div className="flex gap-2">
          {['start', 'stop', 'reset'].map(cmd => (
            <button
              key={cmd}
              onClick={() => { setPublishValue(cmd); topicManager.publish('/hello_world_command', 'std_msgs/String', { data: cmd }); addLog({ level: 'info', source: '/hello_world_command', message: cmd }) }}
              className="px-3 py-1.5 text-xs bg-surface border border-border rounded-lg text-text-secondary hover:text-text-primary hover:border-accent/40 transition-colors"
            >
              {cmd}
            </button>
          ))}
        </div>
      </div>

      {/* Subscriber table */}
      <div className="space-y-2">
        {topics.map(topic => (
          <div key={topic.name} className="bg-surface-2 rounded-xl border border-border overflow-hidden">
            <button
              onClick={() => setExpanded(expanded === topic.name ? null : topic.name)}
              className="w-full flex items-center justify-between px-4 py-3 hover:bg-surface transition-colors"
            >
              <div className="flex items-center gap-3">
                <Radio className={cn('w-3.5 h-3.5', topic.messageCount > 0 ? 'text-success' : 'text-muted')} />
                <span className="text-sm font-mono text-text-primary">{topic.name}</span>
                <span className="text-xs text-muted">{topic.type}</span>
              </div>
              <div className="flex items-center gap-4">
                <span className="text-xs font-mono text-text-secondary">{topic.messageCount} msgs</span>
                <span className="text-xs font-mono text-muted">{formatTimestamp(topic.lastTimestamp)}</span>
                {expanded === topic.name ? <ChevronUp className="w-4 h-4 text-muted" /> : <ChevronDown className="w-4 h-4 text-muted" />}
              </div>
            </button>

            {expanded === topic.name && (
              <div className="px-4 pb-4 border-t border-border">
                <div className="bg-background rounded-lg p-3 mt-3 font-mono text-xs">
                  <p className="text-muted">Last message:</p>
                  <p className="text-text-primary mt-1 break-all">{topic.lastMessage || '(none)'}</p>
                </div>
              </div>
            )}
          </div>
        ))}
      </div>
    </motion.div>
  )
}
