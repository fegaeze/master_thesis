import { useROS } from "../hooks/useROS";

const Alert = () => {
    const { error } = useROS();

    return (
        <>
            {error && (
                <>
                    <div className="flex absolute top-0 right-0 py-6 px-8 mx-5 my-7 text-sm text-red-800 rounded-lg bg-red-100 dark:bg-gray-800 dark:text-red-500" role="alert">
                        <svg className="flex-shrink-0 inline w-4 h-4 mr-3 mt-[2px]" aria-hidden="true" xmlns="http://www.w3.org/2000/svg" fill="currentColor" viewBox="0 0 20 20">
                            <path d="M10 .5a9.5 9.5 0 1 0 9.5 9.5A9.51 9.51 0 0 0 10 .5ZM9.5 4a1.5 1.5 0 1 1 0 3 1.5 1.5 0 0 1 0-3ZM12 15H8a1 1 0 0 1 0-2h1v-3H8a1 1 0 0 1 0-2h2a1 1 0 0 1 1 1v4h1a1 1 0 0 1 0 2Z"/>
                        </svg>
                        <div>
                            <span className="font-medium">Seems like there&rsquo;s something wrong with your connection:</span>
                            <ul className="mt-1.5 ml-4 list-disc list-inside">
                                <li>Confirm that the WebSocket URL is accurate</li>
                                <li>Confirm that you are running go1_web_interface/app.launch</li>
                            </ul>
                        </div>
                    </div>
                </>
            )}
        </>
    );
}

export default Alert;