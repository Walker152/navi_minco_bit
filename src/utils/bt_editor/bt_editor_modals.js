        // --- Modals ---
        const ROSConnectionModal = ({ config, onConnect, onClose, status, topics }) => {
            const [url, setUrl] = useState(config.url || 'ws://localhost:9090');
            return (
                <div className="fixed inset-0 bg-black/50 z-50 flex items-center justify-center">
                    <div className="bg-white p-6 rounded-lg shadow-lg w-96">
                        <h3 className="font-bold mb-4 text-slate-700 flex items-center gap-2"><Icons.Wifi className="text-blue-500"/> ROS2 Bridge 连接</h3>
                        <div className="mb-4">
                            <label className="block text-xs text-gray-500 mb-1">WebSocket URL</label>
                            <input className="w-full border p-2 rounded text-sm font-mono" value={url} onChange={(e) => setUrl(e.target.value)} placeholder="ws://localhost:9090"/>
                        </div>
                        <div className="flex items-center gap-2 mb-4 text-sm">
                            Status: <span className={status === 'Connected' ? 'text-green-600 font-bold' : status === 'Error' ? 'text-red-600 font-bold' : 'text-gray-500'}>{status}</span>
                        </div>
                        <div className="flex justify-end gap-2">
                            <button onClick={onClose} className="px-3 py-1 text-sm text-gray-500 hover:bg-gray-100 rounded">关闭</button>
                            <button onClick={() => onConnect(url)} className="px-3 py-1 text-sm bg-blue-600 text-white rounded hover:bg-blue-700">
                                {status === 'Connected' ? '重新连接' : '连接'}
                            </button>
                        </div>
                    </div>
                </div>
            );
        };

        const CodePreviewModal = ({ files, onClose }) => {
            const [activeTab, setActiveTab] = useState(Object.keys(files)[0]);
            return (
                <div className="fixed inset-0 bg-black/50 z-50 flex items-center justify-center p-10">
                    <div className="bg-white w-full max-w-5xl h-[80vh] rounded-lg shadow-2xl flex flex-col overflow-hidden">
                        <div className="flex items-center justify-between p-4 border-b bg-gray-50">
                            <h3 className="font-bold text-slate-700 flex items-center gap-2"><Icons.FileCode className="text-blue-600" /> C++ 代码生成预览</h3>
                            <div className="flex gap-2">
                                <button onClick={() => { Object.entries(files).forEach(([name, content]) => downloadFile(name, content)); }} className="flex items-center gap-1 bg-green-600 text-white px-3 py-1 rounded text-sm hover:bg-green-700"><Icons.Download size={14} /> 下载所有文件</button>
                                <button onClick={onClose} className="p-1 hover:bg-gray-200 rounded"><Icons.X size={20}/></button>
                            </div>
                        </div>
                        <div className="flex border-b bg-gray-100 overflow-x-auto">{Object.keys(files).map(fileName => (<button key={fileName} onClick={() => setActiveTab(fileName)} className={`px-4 py-2 text-xs font-mono border-r whitespace-nowrap ${activeTab === fileName ? 'bg-white text-blue-600 font-bold border-t-2 border-t-blue-500' : 'text-gray-600 hover:bg-gray-200'}`}>{fileName}</button>))}</div>
                        <div className="flex-1 overflow-hidden relative group"><pre className="h-full overflow-auto p-4 text-xs font-mono bg-[#1e1e1e] text-gray-300">{files[activeTab]}</pre></div>
                    </div>
                </div>
            );
        };

