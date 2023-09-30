import { BrowserRouter, Route, Routes } from "react-router-dom";

import { ROS } from "./hooks/useROS";
import ConnectedRoute from "./utils/ConnectedRoutes";
import ControlPage from "./layout/ControlPage";
import LoginPage from "./layout/LoginPage";
import Alert from "./components/Alert";


const App = () => {
  return (
    <ROS>
      <Alert />
      <BrowserRouter>
        <Routes>
          <Route path="/dashboard" element={
            <ConnectedRoute>
              <ControlPage />
            </ConnectedRoute>
          } />
          <Route path="/" element={<LoginPage />} />
        </Routes>
      </BrowserRouter>
    </ROS>
  );
};

export default App;
