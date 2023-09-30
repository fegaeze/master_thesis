import React, { useEffect } from "react";
import { useNavigate } from "react-router-dom"
import { useROS } from "../hooks/useROS";

const LoginPage = () => {
  const navigate = useNavigate();
  const { url, changeUrl, handleConnect, isConnected } = useROS();

  useEffect(() => {
    if(isConnected) navigate("/dashboard");
  }, [isConnected]);

  const handleSubmit = (e) => {
    e.preventDefault();
    handleConnect();
  }

  return (
    <>
      <div className="flex min-h-full -mt-2 flex-1 flex-col justify-center px-6 py-12 lg:px-8">
        <div className="sm:mx-auto sm:w-full sm:max-w-sm">
          <h2 className="text-3xl text-center font-bold tracking-tight text-black sm:text-4xl">
            Go1 Control Interface
          </h2>
          <p className="mt-3 text-center text-md leading-7 text-gray-900">
            Enter your WebSocket URL to establish connection with the robot.
          </p>
        </div>

        <div className="mt-8 sm:mx-auto sm:w-full sm:max-w-sm">
          <form className="space-y-6" onSubmit={handleSubmit}>
            <div>
              <label
                htmlFor="rosbridge_url"
                className="block text-sm font-medium leading-6 text-gray-900"
              >
                ROSBridge URL
              </label>
              <div className="mt-2">
                <input
                  id="rosbridge_url"
                  name="rosbridge_url"
                  type="url"
                  autoComplete="url"
                  required
									value={url}
									onChange={e => changeUrl(e.target.value)}
                  className="block w-full rounded-md border-0 py-1.5 text-gray-900 shadow-sm ring-1 ring-inset ring-gray-300 placeholder:text-gray-400 focus:ring-2 focus:ring-inset focus:ring-indigo-600 sm:text-sm sm:leading-6"
                />
              </div>
            </div>

            <div>
              <button
                className="inline-flex items-center w-full justify-center rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold leading-6 text-white shadow-sm hover:bg-indigo-500 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
              >
                Connect
              </button>
            </div>
          </form>
        </div>
      </div>
    </>
  );
};

export default LoginPage;
