import ROSLIB from 'roslib'
import { rosConnection } from './RosConnection'

class ServiceManager {
  call(
    name: string,
    serviceType: string,
    request: Record<string, unknown> = {},
  ): Promise<unknown> {
    return new Promise((resolve, reject) => {
      const ros = rosConnection.getRos()
      if (!ros) {
        reject(new Error('ROS not connected'))
        return
      }

      const svc = new ROSLIB.Service({ ros, name, serviceType })
      const req = new ROSLIB.ServiceRequest(request)

      svc.callService(req, resolve, (err: string) => reject(new Error(err)))
    })
  }
}

export const serviceManager = new ServiceManager()
export default ServiceManager
