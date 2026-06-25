import { Header } from './Header'
import { Sidebar } from './Sidebar'
import { useStore } from '@/store/store'
import { Dashboard } from '@/pages/Dashboard'
import { CamerasPage } from '@/pages/Cameras'
import { TopicsPage } from '@/pages/Topics'
import { ServicesPage } from '@/pages/Services'
import { ActionsPage } from '@/pages/Actions'
import { SkillsPage } from '@/pages/Skills'
import { LogsPage } from '@/pages/Logs'

const PAGE_MAP: Record<string, React.ReactNode> = {
  dashboard: <Dashboard />,
  cameras:   <CamerasPage />,
  topics:    <TopicsPage />,
  services:  <ServicesPage />,
  actions:   <ActionsPage />,
  skills:    <SkillsPage />,
  logs:      <LogsPage />,
}

export function MainLayout() {
  const activePage = useStore(s => s.activePage)

  return (
    <div className="flex flex-col h-screen bg-background text-text-primary overflow-hidden">
      <Header />
      <div className="flex flex-1 overflow-hidden">
        <Sidebar />
        <main className="flex-1 overflow-y-auto p-6">
          {PAGE_MAP[activePage] ?? <Dashboard />}
        </main>
      </div>
    </div>
  )
}
