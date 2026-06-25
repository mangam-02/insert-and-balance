import { type ClassValue, clsx } from 'clsx'
import { twMerge } from 'tailwind-merge'

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}

export function formatTimestamp(date: Date | null): string {
  if (!date) return '—'
  return date.toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

export function formatDuration(ms: number): string {
  if (ms < 1000) return `${ms}ms`
  if (ms < 60000) return `${(ms / 1000).toFixed(1)}s`
  return `${Math.floor(ms / 60000)}m ${Math.floor((ms % 60000) / 1000)}s`
}

export function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max)
}

export function truncate(str: string, maxLen = 80): string {
  return str.length > maxLen ? str.slice(0, maxLen) + '…' : str
}

// Franka joint limits in radians
export const JOINT_LIMITS = [
  { min: -2.8973, max: 2.8973 },
  { min: -1.7628, max: 1.7628 },
  { min: -2.8973, max: 2.8973 },
  { min: -3.0718, max: -0.0698 },
  { min: -2.8973, max: 2.8973 },
  { min: -0.0175, max: 3.7525 },
  { min: -2.8973, max: 2.8973 },
]

export function jointProgress(angle: number, idx: number): number {
  const { min, max } = JOINT_LIMITS[idx] ?? { min: -Math.PI, max: Math.PI }
  return ((angle - min) / (max - min)) * 100
}
