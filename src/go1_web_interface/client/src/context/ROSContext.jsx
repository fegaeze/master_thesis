import { createContext, useState } from "react";
import ROSLIB from "roslib";

const rosObj = {
  ROS: new ROSLIB.Ros(),
  url: "ws://localhost:8080",
  isConnected: false,
  error: null,
  topics: [],
  services: [],
  listeners: [],
};

const ROSContext = createContext([{}, () => {}]);

const ROSProvider = (props) => {
  const [ros, setROS] = useState(rosObj);
  return (
    <ROSContext.Provider value={[ros, setROS]}>
      {props.children}
    </ROSContext.Provider>
  );
};

export { ROSContext, ROSProvider };
