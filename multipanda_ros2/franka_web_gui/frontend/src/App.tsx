import { useRos } from '@/hooks/useRos'
import { useMockData } from '@/hooks/useMockData'
import { MainLayout } from '@/components/layout/MainLayout'

export default function App() {
  useRos()
  useMockData()

  return <MainLayout />
}
