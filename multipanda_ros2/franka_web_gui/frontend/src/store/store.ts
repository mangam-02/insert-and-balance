import { create } from 'zustand'
import type {
  ConnectionStatus,
  JointState,
  FrankaState,
  LogEntry,
  TopicInfo,
  ActionGoal,
  ServiceCall,
  CameraStream,
  Skill,
} from '@/types/ros.types'

interface AppState {
  // Connection
  connectionStatus: ConnectionStatus
  bridgeUrl: string
  setConnectionStatus: (s: ConnectionStatus) => void
  setBridgeUrl: (url: string) => void

  // Joint / Robot state
  jointState: JointState | null
  frankaState: FrankaState | null
  robotOnline: boolean
  setJointState: (s: JointState) => void
  setFrankaState: (s: FrankaState) => void
  setRobotOnline: (v: boolean) => void

  // Topics
  topics: Record<string, TopicInfo>
  updateTopic: (name: string, partial: Partial<TopicInfo>) => void

  // Actions
  actionGoals: ActionGoal[]
  upsertActionGoal: (goal: ActionGoal) => void
  cancelActionGoal: (id: string) => void

  // Services
  services: Record<string, ServiceCall>
  updateService: (name: string, partial: Partial<ServiceCall>) => void

  // Cameras
  cameras: CameraStream[]
  updateCamera: (topic: string, partial: Partial<CameraStream>) => void

  // Skills
  skills: Skill[]
  updateSkill: (id: string, partial: Partial<Skill>) => void

  // Logs
  logs: LogEntry[]
  addLog: (entry: Omit<LogEntry, 'id' | 'timestamp'>) => void
  clearLogs: () => void

  // Navigation
  activePage: string
  setActivePage: (page: string) => void
}

const DEFAULT_SKILLS: Skill[] = [
  { id: 'home', name: 'Home', description: 'Move to home pose', icon: 'Home', topic: '/franka/skill', payload: 'home', status: 'idle' },
  { id: 'pick', name: 'Pick', description: 'Pick object from scene', icon: 'Hand', topic: '/franka/skill', payload: 'pick', status: 'idle' },
  { id: 'place', name: 'Place', description: 'Place object at target', icon: 'PackageOpen', topic: '/franka/skill', payload: 'place', status: 'idle' },
  { id: 'grasp', name: 'Grasp', description: 'Close gripper to grasp', icon: 'Grip', topic: '/franka/skill', payload: 'grasp', status: 'idle' },
  { id: 'release', name: 'Release', description: 'Open gripper to release', icon: 'HandOpen', topic: '/franka/skill', payload: 'release', status: 'idle' },
  { id: 'wave', name: 'Wave', description: 'Perform a wave gesture', icon: 'Waves', topic: '/franka/skill', payload: 'wave', status: 'idle' },
  { id: 'handshake', name: 'Handshake', description: 'Perform handshake motion', icon: 'Handshake', topic: '/franka/skill', payload: 'handshake', status: 'idle' },
]

const DEFAULT_CAMERAS: CameraStream[] = [
  { topic: '/camera/color/image_compressed', label: 'Color Camera', active: false, fps: 0, width: 1280, height: 720, imageDataUrl: null },
  { topic: '/camera/depth/image_compressed', label: 'Depth Camera', active: false, fps: 0, width: 1280, height: 720, imageDataUrl: null },
]

const DEFAULT_SERVICES: Record<string, ServiceCall> = {
  '/robot/start': { name: '/robot/start', status: 'idle', lastResult: '', lastCallTime: null },
  '/robot/stop': { name: '/robot/stop', status: 'idle', lastResult: '', lastCallTime: null },
  '/robot/home': { name: '/robot/home', status: 'idle', lastResult: '', lastCallTime: null },
  '/franka/error_recovery': { name: '/franka/error_recovery', status: 'idle', lastResult: '', lastCallTime: null },
}

export const useStore = create<AppState>((set) => ({
  connectionStatus: 'disconnected',
  bridgeUrl: 'ws://localhost:9090',
  setConnectionStatus: (s) => set({ connectionStatus: s }),
  setBridgeUrl: (url) => set({ bridgeUrl: url }),

  jointState: null,
  frankaState: null,
  robotOnline: false,
  setJointState: (s) => set({ jointState: s }),
  setFrankaState: (s) => set({ frankaState: s }),
  setRobotOnline: (v) => set({ robotOnline: v }),

  topics: {
    '/hello_world': { name: '/hello_world', type: 'std_msgs/String', rate: 0, lastMessage: '', lastTimestamp: null, messageCount: 0 },
    '/robot_status': { name: '/robot_status', type: 'std_msgs/String', rate: 0, lastMessage: '', lastTimestamp: null, messageCount: 0 },
    '/joint_states': { name: '/joint_states', type: 'sensor_msgs/JointState', rate: 0, lastMessage: '', lastTimestamp: null, messageCount: 0 },
    '/franka_state_controller/franka_states': { name: '/franka_state_controller/franka_states', type: 'franka_msgs/FrankaState', rate: 0, lastMessage: '', lastTimestamp: null, messageCount: 0 },
  },
  updateTopic: (name, partial) =>
    set((state) => ({
      topics: { ...state.topics, [name]: { ...state.topics[name], ...partial } },
    })),

  actionGoals: [],
  upsertActionGoal: (goal) =>
    set((state) => {
      const existing = state.actionGoals.findIndex(g => g.id === goal.id)
      if (existing >= 0) {
        const goals = [...state.actionGoals]
        goals[existing] = goal
        return { actionGoals: goals }
      }
      return { actionGoals: [...state.actionGoals, goal] }
    }),
  cancelActionGoal: (id) =>
    set((state) => ({
      actionGoals: state.actionGoals.map(g => g.id === id ? { ...g, status: 'cancelled' as const } : g),
    })),

  services: DEFAULT_SERVICES,
  updateService: (name, partial) =>
    set((state) => ({
      services: { ...state.services, [name]: { ...state.services[name], ...partial } },
    })),

  cameras: DEFAULT_CAMERAS,
  updateCamera: (topic, partial) =>
    set((state) => ({
      cameras: state.cameras.map(c => c.topic === topic ? { ...c, ...partial } : c),
    })),

  skills: DEFAULT_SKILLS,
  updateSkill: (id, partial) =>
    set((state) => ({
      skills: state.skills.map(s => s.id === id ? { ...s, ...partial } : s),
    })),

  logs: [],
  addLog: (entry) =>
    set((state) => ({
      logs: [
        { ...entry, id: `${Date.now()}-${Math.random()}`, timestamp: new Date() },
        ...state.logs,
      ].slice(0, 500),
    })),
  clearLogs: () => set({ logs: [] }),

  activePage: 'dashboard',
  setActivePage: (page) => set({ activePage: page }),
}))
