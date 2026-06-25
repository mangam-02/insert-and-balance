import { Camera, Wifi } from 'lucide-react'

// Mock camera feed component — swap imageDataUrl from CameraManager for real stream
export function CameraFeed({
  label,
  imageDataUrl,
  fps,
  active,
  compact = false,
}: {
  label: string
  imageDataUrl: string | null
  fps: number
  active: boolean
  compact?: boolean
}) {
  return (
    <div className="bg-background rounded-lg overflow-hidden relative">
      {imageDataUrl ? (
        <img
          src={imageDataUrl}
          alt={label}
          className="w-full h-full object-cover"
        />
      ) : (
        <div className={`flex flex-col items-center justify-center gap-2 ${compact ? 'h-32' : 'h-48'} bg-surface`}>
          <Camera className="w-6 h-6 text-muted" />
          <p className="text-xs text-muted">{active ? 'Awaiting stream…' : 'No signal'}</p>
        </div>
      )}

      {/* Overlay badges */}
      <div className="absolute top-2 left-2 flex items-center gap-1.5 bg-background/80 backdrop-blur-sm px-2 py-1 rounded-md">
        <span className={`w-1.5 h-1.5 rounded-full ${active ? 'bg-success animate-pulse' : 'bg-muted'}`} />
        <span className="text-xs text-text-secondary font-medium">{label}</span>
      </div>

      {active && fps > 0 && (
        <div className="absolute top-2 right-2 bg-background/80 backdrop-blur-sm px-2 py-1 rounded-md">
          <span className="text-xs font-mono text-success">{fps} FPS</span>
        </div>
      )}

      {!active && (
        <div className="absolute bottom-2 right-2">
          <Wifi className="w-3.5 h-3.5 text-muted" />
        </div>
      )}
    </div>
  )
}
