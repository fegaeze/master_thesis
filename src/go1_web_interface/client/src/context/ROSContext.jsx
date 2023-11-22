import { createContext, useState } from "react";
import { Service, Ros } from "roslib";

import { 
  ACTION_SERVICE_TYPE, 
  ACTION_SERVICE_URL, 
  CONTROLLER_TYPE_SERVICE_TYPE, 
  CONTROLLER_TYPE_SERVICE_URL, 
  PID_TUNING_SERVICE_TYPE,
  PID_TUNING_SERVICE_URL,
  ROSBRIDGE_URL 
} from "../config";

const rosObj = {
  ROS: new Ros(),
  url: ROSBRIDGE_URL,
  isConnected: false,
  error: null,
};

const ROSContext = createContext([{}, () => {}]);

const ROSProvider = (props) => {
  const [ros, setROS] = useState(rosObj);

  const actionService = new Service({
    ros : ros.ROS,
    name : ACTION_SERVICE_URL,
    serviceType : ACTION_SERVICE_TYPE,
  });

  const controllerTypeService = new Service({
    ros : ros.ROS,
    name : CONTROLLER_TYPE_SERVICE_URL,
    serviceType : CONTROLLER_TYPE_SERVICE_TYPE,
  });

  const pidTuningService = new Service({
    ros : ros.ROS,
    name : PID_TUNING_SERVICE_URL,
    serviceType : PID_TUNING_SERVICE_TYPE,
  });

  return (
    <ROSContext.Provider value={[
      ros, 
      actionService, 
      controllerTypeService,
      pidTuningService, 
      setROS
    ]}>
      {props.children}
    </ROSContext.Provider>
  );
};

export { ROSContext, ROSProvider };
