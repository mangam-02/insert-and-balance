export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected' | 'error'

export interface JointState {
  name: string[]
  position: number[]
  velocity: number[]
  effort: number[]
  header: { stamp: { sec: number; nanosec: number }; frame_id: string }
}

export interface FrankaState {
  q: number[]
  dq: number[]
  tau_j: number[]
  o_t_ee: number[]
  cartesian_collision: number[]
  joint_contact: number[]
}

export interface RobotStatus {
  mode: string
  connected: boolean
  error_code: number
  message: string
}

export interface LogEntry {
  id: string
  timestamp: Date
  level: 'info' | 'warn' | 'error' | 'success'
  source: string
  message: string
}

export interface TopicInfo {
  name: string
  type: string
  rate: number
  lastMessage: string
  lastTimestamp: Date | null
  messageCount: number
}

export interface ActionGoal {
  id: string
  name: string
  status: 'idle' | 'pending' | 'executing' | 'succeeded' | 'failed' | 'cancelled'
  progress: number
  feedback: string
  startTime: Date | null
}

export interface ServiceCall {
  name: string
  status: 'idle' | 'calling' | 'success' | 'error'
  lastResult: string
  lastCallTime: Date | null
}

export interface CameraStream {
  topic: string
  label: string
  active: boolean
  fps: number
  width: number
  height: number
  imageDataUrl: string | null
}

export interface Skill {
  id: string
  name: string
  description: string
  icon: string
  topic: string
  payload: string
  status: 'idle' | 'running' | 'success' | 'error'
}
