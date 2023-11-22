import { createContext, useState } from "react";
import { Service, Ros } from "roslib";

import { ACTION_SERVICE_TYPE, ACTION_SERVICE_URL, CONTROLLER_SERVICE_URL, CONTROLLER_SERVICE_TYPE, ROSBRIDGE_URL } from "../config";

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

  const controllerService = new Service({
    ros : ros.ROS,
    name : CONTROLLER_SERVICE_URL,
    serviceType : CONTROLLER_SERVICE_TYPE,
  });

  return (
    <ROSContext.Provider value={[ros, actionService, controllerService, setROS]}>
      {props.children}
    </ROSContext.Provider>
  );
};

export { ROSContext, ROSProvider };
