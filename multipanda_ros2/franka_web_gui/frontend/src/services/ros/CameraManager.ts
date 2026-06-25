import { topicManager } from './TopicManager'

type FrameCallback = (dataUrl: string, fps: number) => void

interface StreamEntry {
  unsubscribe: () => void
  callbacks: Set<FrameCallback>
  lastFrame: number
  fps: number
}

class CameraManager {
  private streams = new Map<string, StreamEntry>()

  subscribe(topic: string, cb: FrameCallback): () => void {
    let entry = this.streams.get(topic)
    if (!entry) {
      const callbacks: Set<FrameCallback> = new Set()
      let lastFrame = 0
      let fps = 0

      const unsub = topicManager.subscribe(
        topic,
        'sensor_msgs/CompressedImage',
        (msg) => {
          const now = Date.now()
          fps = lastFrame ? Math.round(1000 / (now - lastFrame)) : 0
          lastFrame = now

          const data = (msg as { data: string }).data
          const dataUrl = `data:image/jpeg;base64,${data}`
          callbacks.forEach(fn => fn(dataUrl, fps))
        },
      )

      entry = { unsubscribe: unsub, callbacks, lastFrame, fps }
      this.streams.set(topic, entry)
    }

    entry.callbacks.add(cb)
    return () => {
      entry!.callbacks.delete(cb)
      if (entry!.callbacks.size === 0) {
        entry!.unsubscribe()
        this.streams.delete(topic)
      }
    }
  }
}

export const cameraManager = new CameraManager()
export default CameraManager
