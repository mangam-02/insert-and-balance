import { useRef, useEffect, useState } from 'react'
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer } from 'recharts'
import { useStore } from '@/store/store'
import { radToDeg } from '@/utils/helpers'

const COLORS = ['#3b82f6', '#22c55e', '#f59e0b', '#ef4444', '#a855f7', '#06b6d4', '#ec4899']
const MAX_POINTS = 60

export function JointChart() {
  const jointState = useStore(s => s.jointState)
  const [history, setHistory] = useState<Record<string, number>[]>([])
  const tickRef = useRef(0)

  useEffect(() => {
    if (!jointState) return
    tickRef.current++
    const point: Record<string, number> = { t: tickRef.current }
    jointState.position.forEach((pos, i) => {
      point[`J${i + 1}`] = parseFloat(radToDeg(pos).toFixed(2))
    })
    setHistory(prev => [...prev.slice(-MAX_POINTS + 1), point])
  }, [jointState])

  return (
    <div className="bg-surface-2 rounded-xl border border-border p-5">
      <div className="flex items-center justify-between mb-4">
        <span className="text-text-secondary text-xs font-medium uppercase tracking-wider">Joint Positions — Live</span>
        <span className="text-xs text-muted">[deg]</span>
      </div>
      <ResponsiveContainer width="100%" height={160}>
        <LineChart data={history} margin={{ top: 4, right: 4, left: -20, bottom: 0 }}>
          <XAxis dataKey="t" hide />
          <YAxis tick={{ fontSize: 10, fill: '#6b7280' }} />
          <Tooltip
            contentStyle={{ background: '#1a1a24', border: '1px solid #2a2a3a', borderRadius: 8, fontSize: 12 }}
            labelStyle={{ color: '#94a3b8' }}
          />
          {Array.from({ length: 7 }, (_, i) => (
            <Line
              key={i}
              type="monotone"
              dataKey={`J${i + 1}`}
              stroke={COLORS[i]}
              dot={false}
              strokeWidth={1.5}
              isAnimationActive={false}
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
  )
}
