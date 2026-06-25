import ROSLIB from 'roslib'
import { rosConnection } from './RosConnection'

export type ActionStatus = 'idle' | 'pending' | 'executing' | 'succeeded' | 'failed' | 'cancelled'

interface ActionCallbacks {
  onFeedback?: (feedback: unknown) => void
  onResult?: (result: unknown) => void
  onStatus?: (status: ActionStatus) => void
}

class ActionManager {
  private activeGoals = new Map<string, ROSLIB.ActionClient>()

  send(
    serverName: string,
    actionName: string,
    goal: Record<string, unknown>,
    cbs: ActionCallbacks = {},
  ): string {
    const ros = rosConnection.getRos()
    if (!ros) return ''

    const client = new ROSLIB.ActionClient({ ros, serverName, actionName })
    const goalMsg = new ROSLIB.Goal({ actionClient: client, goalMessage: goal })
    const id = `${serverName}-${Date.now()}`
    this.activeGoals.set(id, client)

    cbs.onStatus?.('pending')

    goalMsg.on('feedback', (fb: unknown) => {
      cbs.onStatus?.('executing')
      cbs.onFeedback?.(fb)
    })

    goalMsg.on('result', (res: unknown) => {
      cbs.onStatus?.('succeeded')
      cbs.onResult?.(res)
      this.activeGoals.delete(id)
    })

    goalMsg.send()
    return id
  }

  cancel(id: string): void {
    const client = this.activeGoals.get(id)
    if (client) {
      client.cancel()
      this.activeGoals.delete(id)
    }
  }
}

export const actionManager = new ActionManager()
export default ActionManager
