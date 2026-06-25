import { Radio } from 'lucide-react'
import { useStore } from '@/store/store'
import { formatTimestamp, truncate } from '@/utils/helpers'

export function TopicMonitor() {
  const topics = useStore(s => Object.values(s.topics))

  return (
    <div className="bg-surface-2 rounded-xl border border-border p-5 flex flex-col gap-4">
      <div className="flex items-center justify-between">
        <span className="text-text-secondary text-xs font-medium uppercase tracking-wider">Topic Monitor</span>
        <Radio className="w-4 h-4 text-muted" />
      </div>

      <div className="space-y-2">
        {topics.map(topic => (
          <div key={topic.name} className="bg-background rounded-lg px-3 py-2.5">
            <div className="flex items-center justify-between">
              <span className="text-xs font-mono text-accent">{topic.name}</span>
              <span className={`text-xs font-mono ${topic.messageCount > 0 ? 'text-success' : 'text-muted'}`}>
                {topic.rate > 0 ? `${topic.rate} Hz` : (topic.messageCount > 0 ? `${topic.messageCount} msgs` : '—')}
              </span>
            </div>
            <div className="flex items-center justify-between mt-1">
              <span className="text-xs text-text-secondary truncate max-w-[200px]">
                {topic.lastMessage ? truncate(topic.lastMessage, 40) : 'No messages'}
              </span>
              <span className="text-xs text-muted font-mono">{formatTimestamp(topic.lastTimestamp)}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}
