import { motion } from 'framer-motion'
import { CameraFeed } from '@/components/dashboard/CameraFeed'
import { useStore } from '@/store/store'
import { Camera, RefreshCw } from 'lucide-react'

export function CamerasPage() {
  const cameras = useStore(s => s.cameras)

  return (
    <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-xl font-semibold text-text-primary">Camera Feeds</h1>
          <p className="text-sm text-text-secondary mt-0.5">Live sensor streams from the Franka workspace</p>
        </div>
        <div className="flex items-center gap-2 text-xs text-muted">
          <Camera className="w-4 h-4" />
          <span>{cameras.length} streams configured</span>
        </div>
      </div>

      {/* Main color camera */}
      <div className="bg-surface-2 rounded-xl border border-border overflow-hidden">
        <div className="flex items-center justify-between px-4 py-3 border-b border-border">
          <div className="flex items-center gap-2">
            <span className={`w-2 h-2 rounded-full ${cameras[0]?.active ? 'bg-success animate-pulse' : 'bg-muted'}`} />
            <span className="text-sm font-medium text-text-primary">{cameras[0]?.topic}</span>
          </div>
          <span className="text-xs font-mono text-muted">{cameras[0]?.width}×{cameras[0]?.height}</span>
        </div>
        <div className="p-4">
          <CameraFeed
            label={cameras[0]?.label ?? 'Color'}
            imageDataUrl={cameras[0]?.imageDataUrl ?? null}
            fps={cameras[0]?.fps ?? 0}
            active={cameras[0]?.active ?? false}
          />
        </div>
      </div>

      {/* Depth camera */}
      <div className="bg-surface-2 rounded-xl border border-border overflow-hidden">
        <div className="flex items-center justify-between px-4 py-3 border-b border-border">
          <div className="flex items-center gap-2">
            <span className={`w-2 h-2 rounded-full ${cameras[1]?.active ? 'bg-success animate-pulse' : 'bg-muted'}`} />
            <span className="text-sm font-medium text-text-primary">{cameras[1]?.topic}</span>
          </div>
          <span className="text-xs font-mono text-muted">{cameras[1]?.width}×{cameras[1]?.height}</span>
        </div>
        <div className="p-4">
          <CameraFeed
            label={cameras[1]?.label ?? 'Depth'}
            imageDataUrl={cameras[1]?.imageDataUrl ?? null}
            fps={cameras[1]?.fps ?? 0}
            active={cameras[1]?.active ?? false}
            compact
          />
        </div>
      </div>

      <div className="flex items-center gap-2 text-xs text-muted bg-surface-2 rounded-lg px-4 py-3 border border-border">
        <RefreshCw className="w-3.5 h-3.5" />
        <span>Streams use <code className="text-accent">sensor_msgs/CompressedImage</code> via rosbridge. Switch to WebRTC for lower latency.</span>
      </div>
    </motion.div>
  )
}
