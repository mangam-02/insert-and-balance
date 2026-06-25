import ROSLIB from 'roslib'
import { rosConnection } from './RosConnection'

type MessageCallback = (message: unknown) => void

interface SubscriptionEntry {
  topic: ROSLIB.Topic
  callbacks: Set<MessageCallback>
}

class TopicManager {
  private subscriptions = new Map<string, SubscriptionEntry>()

  subscribe(name: string, messageType: string, cb: MessageCallback): () => void {
    const ros = rosConnection.getRos()
    if (!ros) return () => {}

    let entry = this.subscriptions.get(name)
    if (!entry) {
      const topic = new ROSLIB.Topic({ ros, name, messageType })
      entry = { topic, callbacks: new Set() }
      this.subscriptions.set(name, entry)
      topic.subscribe((msg) => {
        entry!.callbacks.forEach(fn => fn(msg))
      })
    }

    entry.callbacks.add(cb)
    return () => {
      entry!.callbacks.delete(cb)
      if (entry!.callbacks.size === 0) {
        entry!.topic.unsubscribe()
        this.subscriptions.delete(name)
      }
    }
  }

  publish(name: string, messageType: string, payload: unknown): void {
    const ros = rosConnection.getRos()
    if (!ros) return
    const topic = new ROSLIB.Topic({ ros, name, messageType })
    const msg = new ROSLIB.Message(payload as Record<string, unknown>)
    topic.publish(msg)
  }

  unsubscribeAll(): void {
    this.subscriptions.forEach(entry => entry.topic.unsubscribe())
    this.subscriptions.clear()
  }
}

export const topicManager = new TopicManager()
export default TopicManager
