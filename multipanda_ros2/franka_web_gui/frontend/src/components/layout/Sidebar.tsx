import {
  LayoutDashboard, Camera, Radio, Wrench, Play, BookOpen, Terminal, ChevronRight,
} from 'lucide-react'
import { motion } from 'framer-motion'
import { useStore } from '@/store/store'
import { cn } from '@/utils/helpers'

const NAV_ITEMS = [
  { id: 'dashboard', label: 'Dashboard',  icon: LayoutDashboard },
  { id: 'cameras',   label: 'Cameras',    icon: Camera },
  { id: 'topics',    label: 'Topics',     icon: Radio },
  { id: 'services',  label: 'Services',   icon: Wrench },
  { id: 'actions',   label: 'Actions',    icon: Play },
  { id: 'skills',    label: 'Skills',     icon: BookOpen },
  { id: 'logs',      label: 'Logs',       icon: Terminal },
]

export function Sidebar() {
  const { activePage, setActivePage } = useStore()

  return (
    <aside className="w-56 flex flex-col border-r border-border bg-surface h-full">
      <nav className="flex-1 p-3 space-y-0.5">
        {NAV_ITEMS.map(({ id, label, icon: Icon }) => {
          const active = activePage === id
          return (
            <motion.button
              key={id}
              onClick={() => setActivePage(id)}
              whileHover={{ x: 2 }}
              whileTap={{ scale: 0.98 }}
              className={cn(
                'w-full flex items-center gap-3 px-3 py-2.5 rounded-lg text-sm font-medium transition-colors text-left',
                active
                  ? 'bg-accent/15 text-accent border border-accent/20'
                  : 'text-text-secondary hover:text-text-primary hover:bg-surface-2',
              )}
            >
              <Icon className={cn('w-4 h-4 flex-shrink-0', active ? 'text-accent' : '')} />
              <span className="flex-1">{label}</span>
              {active && <ChevronRight className="w-3 h-3 text-accent" />}
            </motion.button>
          )
        })}
      </nav>

      {/* Bottom metadata */}
      <div className="p-4 border-t border-border">
        <p className="text-xs text-muted font-mono">v1.0.0-hackathon</p>
        <p className="text-xs text-muted mt-0.5">ROS2 Humble</p>
      </div>
    </aside>
  )
}
