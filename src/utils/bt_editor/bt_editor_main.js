        // --- Core Editor Component ---
        function BehaviorTreeEditor() {
            // Local Storage Presets
            const [customPresets, setCustomPresets] = useState(() => JSON.parse(localStorage.getItem('rm_bt_presets') || '[]'));
            
            // Panel visibility states
            const [showLibrary, setShowLibrary] = useState(true);
            const [showRightPanel, setShowRightPanel] = useState(true);

            // Generate full NODE_TYPES mapping dynamically
            const getDef = useCallback((type) => {
                if (BASE_NODE_TYPES[type]) return BASE_NODE_TYPES[type];
                const preset = customPresets.find(p => p.type === type);
                if (preset) return { label: preset.label, group: 'custom', baseType: preset.baseType, color: preset.color, icon: preset.icon, desc: preset.desc };
                return { label: type, group: 'unknown', baseType: 'ACTION', color: '#64748b', icon: 'Activity', desc: '未知节点' };
            }, [customPresets]);

            const [nodes, setNodes] = useState([]);
            const [edges, setEdges] = useState([]);
            const [selectedNodeId, setSelectedNodeId] = useState(null);
            const [blackboard, setBlackboard] = useState(INITIAL_BLACKBOARD);
            
            const [view, setView] = useState({ x: 0, y: 0, k: 1 });
            const [isPanning, setIsPanning] = useState(false);
            const [draggingNodeId, setDraggingNodeId] = useState(null);
            const [connectingSourceId, setConnectingSourceId] = useState(null);
            const [mousePos, setMousePos] = useState({ x: 0, y: 0 });
            const [lastMousePos, setLastMousePos] = useState({ x: 0, y: 0 });
            
            const [rightPanelWidth, setRightPanelWidth] = useState(340);
            const [bottomPanelHeight, setBottomPanelHeight] = useState(250);
            const [isResizingPanel, setIsResizingPanel] = useState(false);
            const [isResizingBottom, setIsResizingBottom] = useState(false);
            
            // Modals & States
            const [showCodeModal, setShowCodeModal] = useState(false);
            const [showROSModal, setShowROSModal] = useState(false);
            const [generatedFiles, setGeneratedFiles] = useState({});
            const [isSimulating, setIsSimulating] = useState(false);
            
            // ROS State
            const [ros, setRos] = useState(null);
            const [rosStatus, setRosStatus] = useState('Disconnected');
            const [rosTopics, setRosTopics] = useState([]);
            const [rosConfig, setRosConfig] = useState({ url: 'ws://localhost:9090' });
            
            const fileInputRef = useRef(null);
            const canvasRef = useRef(null);
            const simulationRef = useRef(null);
            const simState = useRef({ stack: [], nodeStatuses: {}, actionTicks: {}, isResetting: false, waitCounter: 0 });
            
            // State refs for simulation closure
            const nodesRef = useRef(nodes);
            const blackboardRef = useRef(blackboard);
            useEffect(() => { nodesRef.current = nodes; }, [nodes]);
            useEffect(() => { blackboardRef.current = blackboard; }, [blackboard]);

            // Init empty root
            useEffect(() => {
                if(nodes.length === 0) {
                    setNodes([{ id: generateId('n'), type: 'ReactiveSequence', x: window.innerWidth/2 - NODE_WIDTH/2, y: 100, data: { label: 'SentryMainLogic', attributes: { name: 'SentryMainLogic' } } }]);
                }
            }, []);

            // --- Save Custom Node Preset ---
            const saveAsPreset = () => {
                if(!selectedNodeId) return;
                const node = nodes.find(n => n.id === selectedNodeId);
                const name = prompt("请输入预制节点名称 (必须是合法的C++类名，如 'MyNavigateNode')", node.data.attributes?.name || node.data.label || 'CustomNode');
                if(!name) return;

                const baseDef = getDef(node.type);
                const newPreset = {
                    id: 'tpl_' + Date.now(),
                    type: name, // The XML tag will be this name
                    baseType: baseDef.baseType || 'ACTION',
                    label: name,
                    group: 'custom',
                    color: '#0d9488', // Teal for custom
                    icon: 'Star',
                    desc: '用户自定义节点组合',
                    dataTemplate: { ...node.data, label: name, attributes: { ...node.data.attributes, name: name } }
                };

                const updated = [...customPresets.filter(p => p.type !== name), newPreset]; // Overwrite if same name
                setCustomPresets(updated);
                localStorage.setItem('rm_bt_presets', JSON.stringify(updated));
                alert(`已保存预制节点: ${name}，可在左侧面板查看！`);
            };

            const deletePreset = (id, e) => {
                e.stopPropagation();
                if(confirm("确定删除此预制节点吗？")) {
                    const updated = customPresets.filter(p => p.id !== id);
                    setCustomPresets(updated);
                    localStorage.setItem('rm_bt_presets', JSON.stringify(updated));
                }
            };

            // --- ROS & Simulation ---
            const connectROS = (url) => {
                setRosConfig({ url });
                if (ros) ros.close();
                const newRos = new ROSLIB.Ros({ url: url });
                newRos.on('connection', () => { setRosStatus('Connected'); newRos.getTopics((r) => setRosTopics(r.topics.map((n, i) => ({ name: n, type: r.types[i] })))); });
                newRos.on('error', () => setRosStatus('Error'));
                newRos.on('close', () => setRosStatus('Disconnected'));
                setRos(newRos);
            };

            const runStep = useCallback(() => {
                const currentNodes = nodesRef.current;
                const nodeMap = new Map(currentNodes.map(n => [n.id, n]));
                const childrenMap = {};
                edges.forEach(edge => { if (!childrenMap[edge.source]) childrenMap[edge.source] = []; childrenMap[edge.source].push(edge.target); });
                Object.keys(childrenMap).forEach(parentId => { childrenMap[parentId].sort((a, b) => (nodeMap.get(a)?.x || 0) - (nodeMap.get(b)?.x || 0)); });

                const state = simState.current;
                
                if (state.isResetting) {
                    state.waitCounter++;
                    if (state.waitCounter > 3) {
                        state.isResetting = false; state.stack = []; state.nodeStatuses = {}; state.actionTicks = {}; state.waitCounter = 0;
                        const root = currentNodes.find(n => !edges.some(e => e.target === n.id)); // Find topological root
                        if (root) state.stack.push({ nodeId: root.id, childIndex: 0 });
                    }
                    setNodes(ns => ns.map(n => ({...n, data: {...n.data, status: state.nodeStatuses[n.id] || 'IDLE'}})));
                    return;
                }

                if (state.stack.length === 0) {
                    const root = currentNodes.find(n => !edges.some(e => e.target === n.id));
                    if (root) state.stack.push({ nodeId: root.id, childIndex: 0 });
                    else return;
                }

                const frame = state.stack[state.stack.length - 1];
                const nodeId = frame.nodeId;
                const node = nodeMap.get(nodeId);
                if (!node) { state.stack.pop(); return; }

                const children = childrenMap[nodeId] || [];
                const def = getDef(node.type);
                const bType = def.baseType;
                let currentStatus = state.nodeStatuses[nodeId] || 'IDLE';

                if (currentStatus === 'IDLE') {
                    state.nodeStatuses[nodeId] = 'RUNNING';
                    setNodes(ns => ns.map(n => n.id === nodeId ? {...n, data: {...n.data, status: 'RUNNING'}} : n));
                    return;
                }

                let result = 'RUNNING';

                if (bType === 'SEQUENCE' || bType === 'FALLBACK') {
                    if (frame.childIndex >= children.length) {
                        result = bType === 'FALLBACK' ? 'FAILURE' : 'SUCCESS';
                    } else {
                        const childId = children[frame.childIndex];
                        const childStatus = state.nodeStatuses[childId] || 'IDLE';
                        if (childStatus === 'IDLE') { state.stack.push({ nodeId: childId, childIndex: 0 }); return; }
                        else if (childStatus === 'RUNNING') return;
                        else {
                            if (bType === 'SEQUENCE') { if (childStatus === 'FAILURE') result = 'FAILURE'; else frame.childIndex++; }
                            else { if (childStatus === 'SUCCESS') result = 'SUCCESS'; else frame.childIndex++; }
                        }
                    }
                } else if (bType === 'ACTION' || bType === 'CONDITION') {
                    if (!state.actionTicks[nodeId]) state.actionTicks[nodeId] = 0;
                    state.actionTicks[nodeId]++;
                    const requiredTicks = bType === 'ACTION' ? 4 : 1; 
                    
                    if (state.actionTicks[nodeId] >= requiredTicks) {
                        // Very basic simulation logic based on Blackboard
                        const mode = node.data.logicMode || 'simple';
                        let pass = true;
                        if (mode === 'simple' && node.data.variable) {
                            const bbItem = blackboardRef.current.find(b => b.key === node.data.variable);
                            if (bType === 'CONDITION') {
                                if (bbItem) {
                                    let tVal = node.data.value;
                                    if (['int','float'].includes(bbItem.type)) tVal = parseFloat(tVal);
                                    if (bbItem.type === 'boolean') tVal = (String(tVal) === 'true');
                                    const op = node.data.op || '==';
                                    if (op === '==') pass = bbItem.value == tVal;
                                    else if (op === '>') pass = bbItem.value > tVal;
                                    else if (op === '<') pass = bbItem.value < tVal;
                                } else pass = false;
                            } else {
                                if (bbItem && node.data.op === '=') {
                                    const newList = [...blackboardRef.current];
                                    const idx = newList.findIndex(b => b.key === node.data.variable);
                                    let val = node.data.value;
                                    if (['int','float'].includes(bbItem.type)) val = parseFloat(val);
                                    if (bbItem.type === 'boolean') val = (String(val) === 'true');
                                    newList[idx].value = val;
                                    setBlackboard(newList);
                                }
                            }
                        }
                        result = pass ? 'SUCCESS' : 'FAILURE';
                        state.actionTicks[nodeId] = 0;
                    }
                } else if (bType === 'DECORATOR') {
                    if (children.length > 0) {
                        const childId = children[0];
                        const childStatus = state.nodeStatuses[childId] || 'IDLE';
                        if (childStatus === 'IDLE') { state.stack.push({ nodeId: childId, childIndex: 0 }); return; }
                        if (childStatus === 'RUNNING') return;
                        result = node.type === 'ForceSuccess' ? 'SUCCESS' : childStatus; // Simplified
                    } else result = 'SUCCESS';
                }

                if (result !== 'RUNNING') {
                    state.nodeStatuses[nodeId] = result;
                    state.stack.pop();
                    if (state.stack.length === 0) state.isResetting = true;
                }
                setNodes(ns => ns.map(n => ({...n, data: {...n.data, status: state.nodeStatuses[n.id] || 'IDLE'}})));
            }, [edges]);

            useEffect(() => {
                if (isSimulating) { simState.current = { stack: [], nodeStatuses: {}, actionTicks: {}, isResetting: false, waitCounter: 0 }; simulationRef.current = setInterval(runStep, 250); }
                return () => { clearInterval(simulationRef.current); if (isSimulating) setNodes(prev => prev.map(n => ({ ...n, data: { ...n.data, status: 'IDLE' } }))); };
            }, [isSimulating, runStep]);

            // --- Interactions ---
            const screenToWorld = useCallback((sx, sy) => {
                if (!canvasRef.current) return { x: 0, y: 0 };
                const rect = canvasRef.current.getBoundingClientRect();
                return { x: (sx - rect.left - view.x) / view.k, y: (sy - rect.top - view.y) / view.k };
            }, [view]);

            const handleGlobalMouseMove = (e) => {
                const worldPos = screenToWorld(e.clientX, e.clientY);
                setMousePos(worldPos);
                if (isPanning) { setView(v => ({ ...v, x: v.x + e.clientX - lastMousePos.x, y: v.y + e.clientY - lastMousePos.y })); setLastMousePos({ x: e.clientX, y: e.clientY }); }
                else if (draggingNodeId) { setNodes(prev => prev.map(n => n.id === draggingNodeId ? { ...n, x: Math.round((worldPos.x - NODE_WIDTH/2)/20)*20, y: Math.round((worldPos.y - 10)/20)*20 } : n)); }
                else if (isResizingPanel) { e.preventDefault(); setRightPanelWidth(Math.max(300, Math.min(document.body.clientWidth - e.clientX, 800))); }
                else if (isResizingBottom) { e.preventDefault(); setBottomPanelHeight(Math.max(150, Math.min(document.body.clientHeight - e.clientY, 600))); }
            };

            const handleGlobalMouseUp = () => { setIsPanning(false); setDraggingNodeId(null); setIsResizingPanel(false); setIsResizingBottom(false); };

            const autoLayout = (rootId, allNodes, allEdges) => {
                const childrenMap = {}; allEdges.forEach(e => { if(!childrenMap[e.source]) childrenMap[e.source] = []; childrenMap[e.source].push(e.target); });
                let leafX = 0; const H_SPACE = 220; const V_SPACE = 140;
                const assignPositions = (nodeId, depth) => {
                    const children = childrenMap[nodeId] || []; let myX = 0;
                    if (children.length === 0) { myX = leafX; leafX += H_SPACE; } 
                    else { let minX = Infinity, maxX = -Infinity; children.forEach(childId => { const cx = assignPositions(childId, depth + 1); minX = Math.min(minX, cx); maxX = Math.max(maxX, cx); }); myX = (minX + maxX) / 2; }
                    const nIdx = allNodes.findIndex(n => n.id === nodeId);
                    if (nIdx > -1) { allNodes[nIdx].x = myX; allNodes[nIdx].y = 50 + depth * V_SPACE; }
                    return myX;
                };
                assignPositions(rootId, 0);
                const rootNode = allNodes.find(n => n.id === rootId);
                const offset = (window.innerWidth / 2) - 150 - (rootNode ? rootNode.x : 0);
                allNodes.forEach(n => { n.x += offset; });
                return allNodes;
            };

            // --- XML Ops ---
            const handleImportXML = (event) => {
                const file = event.target.files[0]; if (!file) return;
                const reader = new FileReader();
                reader.onload = (e) => {
                    const parser = new DOMParser(); const doc = parser.parseFromString(e.target.result, "text/xml");
                    let parsedNodes = []; let parsedEdges = [];
                    const parseElement = (el) => {
                        if(el.nodeType !== 1) return null;
                        const tagName = el.tagName;
                        if(['root', 'BehaviorTree', 'TreeNodesModel', 'SubTree'].includes(tagName) && el.children.length === 1) return parseElement(el.firstElementChild);
                        const id = generateId('n'); const attrs = {}; Array.from(el.attributes).forEach(a => attrs[a.name] = a.value);
                        
                        // Check if it's a known preset to inject logic
                        const preset = customPresets.find(p => p.type === tagName);
                        const nodeData = preset ? JSON.parse(JSON.stringify(preset.dataTemplate)) : { label: attrs.name || tagName };
                        nodeData.attributes = attrs; // Override with actual XML attrs
                        
                        parsedNodes.push({ id, type: tagName, x: 0, y: 0, data: nodeData });
                        Array.from(el.children).forEach(child => { const childId = parseElement(child); if(childId) parsedEdges.push({ id: generateId('e'), source: id, target: childId }); });
                        return id;
                    };
                    const rootId = parseElement(doc.documentElement);
                    if(rootId) {
                        parsedNodes = autoLayout(rootId, parsedNodes, parsedEdges);
                        setNodes(parsedNodes); setEdges(parsedEdges); setView({x: 0, y: 50, k: 0.8});
                    }
                };
                reader.readAsText(file); event.target.value = null;
            };

            const handleExportXML = () => {
                const rootNodes = nodes.filter(n => !edges.some(e => e.target === n.id));
                if(rootNodes.length === 0) { alert("No valid root node found!"); return; }
                const buildXML = (nodeId, indent) => {
                    const node = nodes.find(n => n.id === nodeId); if(!node) return '';
                    let attrsStr = ''; const attrs = node.data.attributes || {};
                    Object.entries(attrs).forEach(([k, v]) => { if (v.trim() !== '') attrsStr += ` ${k}="${v}"`; });
                    if(!attrs.name && node.data.label !== node.type) attrsStr += ` name="${node.data.label}"`;
                    const childrenIds = edges.filter(e => e.source === nodeId).map(e => e.target);
                    childrenIds.sort((a, b) => (nodes.find(n=>n.id===a)?.x || 0) - (nodes.find(n=>n.id===b)?.x || 0));
                    if(childrenIds.length === 0) return `${indent}<${node.type}${attrsStr}/>\n`;
                    let xml = `${indent}<${node.type}${attrsStr}>\n`;
                    childrenIds.forEach(cid => { xml += buildXML(cid, indent + '  '); });
                    xml += `${indent}</${node.type}>\n`; return xml;
                };
                const finalXML = `<root main_tree_to_execute="MainTree">\n  <BehaviorTree ID="MainTree">\n${buildXML(rootNodes[0].id, '    ')}  </BehaviorTree>\n</root>`;
                downloadFile('nav_tree.xml', finalXML);
            };
            
            // --- Code Gen ---
            const handleGenerateCode = () => {
                // A simplified code gen mapping logic to C++ actions/conditions
                let cpp = `#include "auto_generated_nodes.hpp"\n#include <iostream>\nnamespace Sentry_BT {\n`;
                const customNodes = nodes.filter(n => !['SEQUENCE','FALLBACK','ReactiveSequence','ReactiveFallback','ForceSuccess'].includes(n.type));
                const seenTypes = new Set();
                customNodes.forEach(n => {
                    if(seenTypes.has(n.type)) return;
                    seenTypes.add(n.type);
                    const bType = getDef(n.type).baseType;
                    const baseClass = bType === 'ACTION' ? 'BT::SyncActionNode' : 'BT::ConditionNode';
                    cpp += `\n// --- ${n.type} ---\n`;
                    cpp += `class ${n.type} : public ${baseClass} {\npublic:\n  ${n.type}(const std::string& name, const BT::NodeConfiguration& config) : ${baseClass}(name, config) {}\n  static BT::PortsList providedPorts() { return {}; }\n  BT::NodeStatus tick() override {\n    auto blackboard = config().blackboard;\n    // TODO: Implement logic based on Web Editor config\n    return BT::NodeStatus::SUCCESS;\n  }\n};\n`;
                });
                cpp += `\n} // namespace\n`;
                setGeneratedFiles({'auto_generated_nodes.cpp': cpp, 'behavior_tree.xml': "/* Use Export XML button */"});
                setShowCodeModal(true);
            };

            // Node Data Ops
            const updateNodeAttr = (key, value) => {
                setNodes(ns => ns.map(n => {
                    if(n.id === selectedNodeId) {
                        const newAttrs = { ...(n.data.attributes || {}), [key]: value };
                        const newLabel = key === 'name' ? value : n.data.label;
                        return { ...n, data: { ...n.data, label: newLabel, attributes: newAttrs } };
                    }
                    return n;
                }));
            };
            const updateNodeData = (key, value) => { setNodes(ns => ns.map(n => n.id === selectedNodeId ? {...n, data: {...n.data, [key]: value}} : n)); };
            const deleteNode = (id) => { setNodes(ns => ns.filter(n => n.id !== id)); setEdges(es => es.filter(e => e.source !== id && e.target !== id)); if(selectedNodeId === id) setSelectedNodeId(null); };

            const selectedNode = nodes.find(n => n.id === selectedNodeId);
            const selectedDef = selectedNode ? getDef(selectedNode.type) : null;
            const getBezierPath = (x1, y1, x2, y2) => `M ${x1} ${y1} C ${x1} ${y1+60} ${x2} ${y2-60} ${x2} ${y2}`;

            return (
                <div className="flex flex-col h-screen w-full bg-[#f8fafc] text-slate-800 font-sans select-none overflow-hidden" onMouseMove={handleGlobalMouseMove} onMouseUp={handleGlobalMouseUp}>
                    <input type="file" accept=".xml" ref={fileInputRef} onChange={handleImportXML} className="hidden" />
                    {showROSModal && <ROSConnectionModal config={rosConfig} status={rosStatus} topics={rosTopics} onConnect={connectROS} onClose={() => setShowROSModal(false)} />}
                    {showCodeModal && <CodePreviewModal files={generatedFiles} onClose={() => setShowCodeModal(false)} />}

                    {/* TopBar */}
                    <div className="h-14 bg-white border-b flex items-center px-5 justify-between shadow-sm z-30 shrink-0">
                        <div className="flex items-center gap-4">
                            <div className="flex items-center gap-2 font-black text-xl text-slate-800 tracking-tight">
                                <Icons.Zap className="text-amber-500" size={24}/> HERO BT Ultimate
                            </div>
                            
                            <div className="flex gap-1 border-l pl-4 border-gray-200">
                                <button onClick={() => setShowLibrary(!showLibrary)} className={`p-1.5 rounded transition-colors ${showLibrary ? 'bg-indigo-50 text-indigo-600' : 'text-slate-400 hover:bg-slate-100'}`} title="显示/隐藏节点库">
                                    <Icons.PanelLeft size={18}/>
                                </button>
                                <button onClick={() => setShowRightPanel(!showRightPanel)} className={`p-1.5 rounded transition-colors ${showRightPanel ? 'bg-indigo-50 text-indigo-600' : 'text-slate-400 hover:bg-slate-100'}`} title="显示/隐藏属性与黑板面板">
                                    <Icons.PanelRight size={18}/>
                                </button>
                            </div>
                        </div>

                        <div className="flex gap-2">
                            <button onClick={() => setShowROSModal(true)} className={`flex items-center gap-1 px-3 py-1.5 rounded transition font-bold text-sm border ${rosStatus === 'Connected' ? 'bg-green-50 border-green-200 text-green-600' : 'bg-red-50 border-red-200 text-red-600'}`}>
                               {rosStatus === 'Connected' ? <Icons.Wifi size={14}/> : <Icons.WifiOff size={14}/>} ROS2 Bridge
                           </button>
                           <button onClick={() => setIsSimulating(!isSimulating)} className={`flex items-center gap-1 px-3 py-1.5 rounded transition text-sm font-bold border ${isSimulating ? 'bg-amber-100 text-amber-700 border-amber-300' : 'bg-emerald-50 text-emerald-600 border-emerald-200'}`}>
                               {isSimulating ? <><Icons.Square size={14} /> 停止</> : <><Icons.Play size={14} /> 本地仿真</>}
                           </button>
                            <button onClick={handleGenerateCode} className="flex items-center gap-1 px-3 py-1.5 bg-gray-100 text-gray-700 hover:bg-gray-200 rounded font-medium text-sm">
                                <Icons.FileCode size={14}/> C++生成
                            </button>
                            <button onClick={() => fileInputRef.current.click()} className="flex items-center gap-1 px-4 py-1.5 bg-indigo-50 text-indigo-600 hover:bg-indigo-100 border border-indigo-200 rounded font-bold text-sm transition">
                                <Icons.Upload size={14}/> 导入
                            </button>
                            <button onClick={handleExportXML} className="flex items-center gap-1 px-4 py-1.5 bg-blue-600 text-white hover:bg-blue-700 rounded font-bold text-sm shadow-sm transition">
                                <Icons.Download size={14}/> 导出
                            </button>
                        </div>
                    </div>

                    <div className="flex flex-1 overflow-hidden relative">
                        {/* Library Panel (Toggleable) */}
                        {showLibrary && (
                            <div className="w-56 bg-white border-r flex flex-col shadow-sm z-20 shrink-0 overflow-y-auto">
                                <div className="p-3 border-b bg-gray-50 flex justify-between items-center">
                                    <span className="text-xs font-bold text-gray-500 uppercase tracking-wider">节点库</span>
                                    <button onClick={() => { setNodes(autoLayout(nodes.find(n=>!edges.some(e=>e.target===n.id))?.id, [...nodes], edges)); setView({x:0, y:50, k:0.8}); }} className="p-1 hover:bg-gray-200 rounded text-gray-600" title="排版"><Icons.RefreshCw size={14}/></button>
                                </div>
                                
                                <div className="p-2 space-y-4">
                                    {/* Custom Presets */}
                                    {customPresets.length > 0 && (
                                        <div>
                                            <div className="text-[10px] font-bold text-teal-600 mb-2 pl-1 flex items-center gap-1"><Icons.Star size={10}/> 自定义预制 (Custom)</div>
                                            <div className="space-y-1">
                                                {customPresets.map((preset) => (
                                                    <div key={preset.id} draggable onDragStart={(e) => e.dataTransfer.setData('nodeType', preset.type)} className="group flex items-center gap-2 p-2 rounded cursor-grab border border-teal-100 bg-teal-50 hover:bg-teal-100 hover:shadow-sm relative">
                                                        <span style={{color: preset.color}}>{React.createElement(Icons[preset.icon] || Icons.Star, {size: 14})}</span>
                                                        <div className="flex flex-col flex-1 overflow-hidden"><span className="text-xs font-bold text-teal-900 truncate">{preset.label}</span></div>
                                                        <button onClick={(e) => deletePreset(preset.id, e)} className="hidden group-hover:block absolute right-1 text-teal-400 hover:text-red-500 bg-white rounded p-0.5"><Icons.X size={10}/></button>
                                                    </div>
                                                ))}
                                            </div>
                                        </div>
                                    )}

                                    <div>
                                        <div className="text-[10px] font-bold text-gray-400 mb-2 pl-1">控制节点 (Control Flow)</div>
                                        <div className="space-y-1">
                                            {Object.entries(BASE_NODE_TYPES).filter(([_, n]) => n.group === 'std').map(([type, def]) => (
                                                <div key={type} draggable onDragStart={(e) => e.dataTransfer.setData('nodeType', type)} className="flex items-center gap-2 p-2 rounded cursor-grab border border-transparent hover:border-gray-200 hover:bg-gray-50 hover:shadow-sm">
                                                    <span style={{color: def.color}}>{React.createElement(Icons[def.icon] || Icons.Activity, {size: 16})}</span>
                                                    <div className="flex flex-col"><span className="text-sm font-bold text-gray-700">{type}</span><span className="text-[9px] text-gray-400 leading-tight">{def.desc}</span></div>
                                                </div>
                                            ))}
                                        </div>
                                    </div>
                                    
                                    <div>
                                        <div className="text-[10px] font-bold text-blue-500 mb-2 pl-1">RM 动作 (Actions)</div>
                                        <div className="space-y-1">
                                            {Object.entries(BASE_NODE_TYPES).filter(([_, n]) => n.group === 'rm_act').map(([type, def]) => (
                                                <div key={type} draggable onDragStart={(e) => e.dataTransfer.setData('nodeType', type)} className="flex items-center gap-2 p-2 rounded cursor-grab border border-blue-100 bg-blue-50/50 hover:bg-blue-50 hover:shadow-sm">
                                                    <span style={{color: def.color}}>{React.createElement(Icons[def.icon] || Icons.Activity, {size: 14})}</span>
                                                    <div className="flex flex-col truncate"><span className="text-xs font-bold text-gray-700 truncate">{def.label}</span></div>
                                                </div>
                                            ))}
                                        </div>
                                    </div>

                                    <div>
                                        <div className="text-[10px] font-bold text-amber-500 mb-2 pl-1">RM 条件 (Conditions)</div>
                                        <div className="space-y-1">
                                            {Object.entries(BASE_NODE_TYPES).filter(([_, n]) => n.group === 'rm_cond').map(([type, def]) => (
                                                <div key={type} draggable onDragStart={(e) => e.dataTransfer.setData('nodeType', type)} className="flex items-center gap-2 p-2 rounded cursor-grab border border-emerald-100 bg-emerald-50/50 hover:bg-emerald-50 hover:shadow-sm">
                                                    <span style={{color: def.color}}>{React.createElement(Icons[def.icon] || Icons.AlertCircle, {size: 14})}</span>
                                                    <div className="flex flex-col truncate"><span className="text-xs font-bold text-gray-700 truncate">{def.label}</span></div>
                                                </div>
                                            ))}
                                        </div>
                                    </div>
                                </div>
                            </div>
                        )}

                        {/* Infinite Canvas */}
                        <div className="flex-1 bg-[#e2e8f0] relative overflow-hidden" ref={canvasRef} 
                            onMouseDown={(e) => { if(e.button===0||e.button===1){ setIsPanning(true); setLastMousePos({x:e.clientX,y:e.clientY}); setSelectedNodeId(null); } }} 
                            onWheel={(e) => { e.preventDefault(); setView(v => ({ ...v, k: Math.min(2, Math.max(0.2, v.k - e.deltaY * 0.001)) })); }} 
                            onDragOver={(e) => e.preventDefault()} 
                            onDrop={(e) => {
                                e.preventDefault(); const type = e.dataTransfer.getData('nodeType'); if(!type) return;
                                const pos = screenToWorld(e.clientX, e.clientY);
                                const preset = customPresets.find(p => p.type === type);
                                const def = getDef(type);
                                
                                // Instantiate data. Merge preset template if exists.
                                const nodeData = preset ? JSON.parse(JSON.stringify(preset.dataTemplate)) : { label: def.label, attributes: { name: def.label } };
                                setNodes(prev => [...prev, { id: generateId('n'), type, x: pos.x - NODE_WIDTH/2, y: pos.y - 20, data: nodeData }]);
                            }}>
                            
                            <div style={{ transform: `translate(${view.x}px, ${view.y}px) scale(${view.k})`, transformOrigin: '0 0', width: '100%', height: '100%' }}>
                                <div className="absolute pointer-events-none opacity-30" style={{ top: -CANVAS_OFFSET, left: -CANVAS_OFFSET, width: CANVAS_OFFSET*2, height: CANVAS_OFFSET*2, backgroundImage: 'radial-gradient(#64748b 1px, transparent 1px)', backgroundSize: '20px 20px' }} />
                                
                                <svg className="absolute pointer-events-none overflow-visible z-0" style={{ top: -CANVAS_OFFSET, left: -CANVAS_OFFSET, width: CANVAS_OFFSET*2, height: CANVAS_OFFSET*2 }}>
                                    {edges.map(edge => {
                                        const src = nodes.find(n => n.id === edge.source); const tgt = nodes.find(n => n.id === edge.target);
                                        if(!src || !tgt) return null;
                                        return <path key={edge.id} d={getBezierPath(src.x+NODE_WIDTH/2+CANVAS_OFFSET, src.y+NODE_HEIGHT+4+CANVAS_OFFSET, tgt.x+NODE_WIDTH/2+CANVAS_OFFSET, tgt.y-4+CANVAS_OFFSET)} stroke="#94a3b8" strokeWidth="3" fill="none" className="cursor-pointer hover:stroke-red-500 pointer-events-auto" onClick={(e) => { e.stopPropagation(); setEdges(es => es.filter(x => x.id !== edge.id)); }} />;
                                    })}
                                    {connectingSourceId && <path d={getBezierPath(nodes.find(n=>n.id===connectingSourceId).x+NODE_WIDTH/2+CANVAS_OFFSET, nodes.find(n=>n.id===connectingSourceId).y+NODE_HEIGHT+4+CANVAS_OFFSET, mousePos.x+CANVAS_OFFSET, mousePos.y+CANVAS_OFFSET)} stroke="#3b82f6" strokeWidth="3" strokeDasharray="5,5" fill="none" />}
                                </svg>

                                {nodes.map(node => {
                                    const def = getDef(node.type);
                                    const IconComp = Icons[def.icon] || Icons.Activity;
                                    const isSelected = selectedNodeId === node.id;
                                    return (
                                        <div key={node.id} onMouseDown={(e) => { e.stopPropagation(); if(e.button===0){ if(connectingSourceId && connectingSourceId!==node.id){ setEdges(p=>[...p,{id:generateId('e'),source:connectingSourceId,target:node.id}]); setConnectingSourceId(null); return;} setSelectedNodeId(node.id); setDraggingNodeId(node.id); } }} style={{ transform: `translate(${node.x}px, ${node.y}px)`, width: `${NODE_WIDTH}px`, height: `${NODE_HEIGHT}px` }} className={`absolute flex flex-col bg-white rounded-lg border-2 shadow-sm transition-all z-10 ${isSelected ? 'border-blue-500 shadow-md ring-4 ring-blue-500/20' : 'border-gray-200 hover:border-gray-300'} ${node.data.status === 'RUNNING' ? 'border-amber-400 ring-4 ring-amber-400/30' : node.data.status === 'SUCCESS' ? 'border-emerald-500 ring-4 ring-emerald-500/30' : node.data.status === 'FAILURE' ? 'border-red-500 ring-4 ring-red-500/30' : ''}`}>
                                            <div className="flex items-center gap-1.5 px-3 py-1.5 border-b bg-gray-50 rounded-t-md pointer-events-none">
                                                <IconComp size={14} color={def.color} />
                                                <span className="text-[10px] font-black tracking-widest uppercase truncate" style={{color: def.color}}>{node.type}</span>
                                            </div>
                                            <div className="flex-1 flex items-center justify-center p-2 text-center pointer-events-none">
                                                <span className="text-sm font-bold text-slate-700 truncate">{node.data.label}</span>
                                            </div>
                                            <div onMouseDown={(e) => { e.stopPropagation(); if(e.button===0) setConnectingSourceId(p => p===node.id ? null : node.id); }} className={`absolute -bottom-3 left-1/2 -translate-x-1/2 w-6 h-6 flex items-center justify-center cursor-pointer z-20 hover:scale-125 ${connectingSourceId === node.id ? 'scale-125' : ''}`}>
                                                <div className={`w-3.5 h-3.5 bg-white border-2 rounded-full shadow-sm ${connectingSourceId === node.id ? 'border-blue-500 bg-blue-100' : 'border-gray-400 hover:border-blue-500'}`}/>
                                            </div>
                                            <div className="absolute -top-2 left-1/2 -translate-x-1/2 w-3.5 h-3.5 bg-white border-2 border-gray-400 rounded-full z-20 pointer-events-none"/>
                                        </div>
                                    );
                                })}
                            </div>
                        </div>

                        {/* Dual Inspector Panel (Toggleable) */}
                        {showRightPanel && (
                            <div className="flex flex-col z-20 shadow-[-4px_0_15px_rgba(0,0,0,0.05)] border-l h-full relative bg-white" style={{ width: rightPanelWidth }}>
                                <div className="resizer-col" onMouseDown={(e) => { e.preventDefault(); setIsResizingPanel(true); }}></div>
                                
                                <div className="flex-1 overflow-y-auto flex flex-col" style={{ height: `calc(100% - ${bottomPanelHeight}px)` }}>
                                    <div className="p-3 border-b bg-gray-50 flex items-center justify-between sticky top-0 z-10">
                                        <span className="font-bold text-sm text-slate-700">属性配置</span>
                                        {selectedNode && (
                                            <div className="flex gap-1">
                                                {(selectedDef?.baseType === 'ACTION' || selectedDef?.baseType === 'CONDITION') && (
                                                    <button onClick={saveAsPreset} className="text-teal-600 hover:bg-teal-50 p-1.5 rounded border border-teal-200 flex items-center gap-1 text-[10px] font-bold"><Icons.Save size={12}/> 保存预制</button>
                                                )}
                                                <button onClick={() => deleteNode(selectedNode.id)} className="text-red-500 hover:bg-red-50 p-1.5 rounded"><Icons.Trash2 size={16}/></button>
                                            </div>
                                        )}
                                    </div>

                                    <div className="p-4">
                                        {selectedNode ? (
                                            <div className="space-y-6">
                                                {/* Top Banner */}
                                                <div className="p-3 bg-slate-50 border border-slate-200 rounded-lg flex items-center gap-3">
                                                    {React.createElement(Icons[selectedDef?.icon || 'Activity'], {size: 24, color: selectedDef?.color || '#64748b'})}
                                                    <div>
                                                        <div className="font-black text-slate-800">{selectedNode.type}</div>
                                                        <div className="text-[10px] text-slate-500">{selectedDef?.desc || '自定义节点'}</div>
                                                    </div>
                                                </div>

                                                {/* 1. XML 运行属性 (真实导出使用) */}
                                                <div>
                                                    <div className="flex items-center justify-between mb-2">
                                                        <label className="text-[11px] font-bold text-indigo-600 uppercase tracking-wider flex items-center gap-1"><Icons.FileCode size={12}/> XML 运行传参 (Attributes)</label>
                                                        <button onClick={() => updateNodeAttr(`attr_${Object.keys(selectedNode.data.attributes || {}).length}`, 'value')} className="text-[10px] bg-indigo-50 hover:bg-indigo-100 text-indigo-600 px-2 py-1 rounded font-bold">添加 +</button>
                                                    </div>
                                                    <div className="space-y-2 p-3 bg-indigo-50/30 border border-indigo-100 rounded-lg">
                                                        <div className="flex flex-col bg-white p-2 rounded border border-indigo-200 shadow-sm">
                                                            <label className="text-[10px] font-bold text-slate-500 mb-1">name (节点标签名)</label>
                                                            <input type="text" className="w-full text-sm px-2 py-1 border rounded" value={selectedNode.data.attributes?.name || ''} onChange={(e) => {
                                                                const val = e.target.value;
                                                                setNodes(ns => ns.map(n => n.id === selectedNodeId ? { ...n, data: { ...n.data, label: val, attributes: { ...(n.data.attributes || {}), name: val } } } : n));
                                                            }} />
                                                        </div>
                                                        {Object.entries(selectedNode.data.attributes || {}).filter(([k]) => k !== 'name').map(([k, v]) => (
                                                            <div key={k} className="flex gap-2 items-start bg-white p-2 rounded border border-indigo-200 shadow-sm relative group">
                                                                <div className="flex-1 space-y-1">
                                                                    <input type="text" className="w-full text-[10px] font-bold text-indigo-600 bg-transparent border-b border-indigo-200 focus:border-indigo-500 outline-none" placeholder="Attribute Key" value={k} onChange={(e) => { const newAttrs={...selectedNode.data.attributes}; newAttrs[e.target.value]=newAttrs[k]; delete newAttrs[k]; setNodes(ns=>ns.map(n=>n.id===selectedNodeId?{...n,data:{...n.data,attributes:newAttrs}}:n)); }} />
                                                                    <input type="text" className="w-full text-sm bg-transparent border-b border-transparent focus:border-indigo-300 px-1 py-1 outline-none" placeholder="Value (e.g. {health})" value={v} onChange={(e) => updateNodeAttr(k, e.target.value)} />
                                                                </div>
                                                                <button onClick={() => { const newAttrs={...selectedNode.data.attributes}; delete newAttrs[k]; setNodes(ns=>ns.map(n=>n.id===selectedNodeId?{...n,data:{...n.data,attributes:newAttrs}}:n)); }} className="mt-4 text-gray-300 hover:text-red-500"><Icons.Trash2 size={14}/></button>
                                                            </div>
                                                        ))}
                                                    </div>
                                                </div>

                                                {/* 2. Web 仿真 & 源码生成配置 */}
                                                {(selectedDef?.baseType === 'ACTION' || selectedDef?.baseType === 'CONDITION') && (
                                                    <div>
                                                        <div className="flex items-center justify-between mb-2">
                                                            <label className="text-[11px] font-bold text-emerald-600 uppercase tracking-wider flex items-center gap-1"><Icons.Play size={12}/> 仿真与逻辑配置 (Logic)</label>
                                                        </div>
                                                        <div className="p-3 bg-emerald-50/30 border border-emerald-100 rounded-lg space-y-3">
                                                            <div className="flex items-center gap-2 p-1 bg-white rounded border border-emerald-100 shadow-sm">
                                                                {['simple'].map(m => (
                                                                    <button key={m} className={`flex-1 py-1 text-xs rounded bg-emerald-100 text-emerald-700 font-bold`}>简单黑板映射</button>
                                                                ))}
                                                            </div>
                                                            <div className="p-2 bg-white rounded border border-emerald-200 shadow-sm space-y-2">
                                                                <div>
                                                                    <label className="text-[10px] font-bold text-slate-500 block mb-1">关联黑板变量 (Blackboard Variable)</label>
                                                                    <select className="w-full border rounded px-2 py-1.5 text-xs bg-gray-50" value={selectedNode.data.variable || ''} onChange={(e) => updateNodeData('variable', e.target.value)}>
                                                                        <option value="">-- 选择变量 --</option>
                                                                        {blackboard.map(b => <option key={b.key} value={b.key}>{b.key} ({b.type})</option>)}
                                                                    </select>
                                                                </div>
                                                                {selectedNode.data.variable && (
                                                                    <div className="flex gap-2 pt-2 border-t border-gray-100">
                                                                        <div className="w-1/3">
                                                                            <label className="text-[10px] font-bold text-slate-500 block mb-1">操作 (Op)</label>
                                                                            <select className="w-full border rounded px-2 py-1.5 text-xs bg-gray-50" value={selectedNode.data.op || '=='} onChange={(e) => updateNodeData('op', e.target.value)}>
                                                                                {selectedDef.baseType === 'CONDITION' ? <><option value="==">==</option><option value=">">&gt;</option><option value="<">&lt;</option></> : <><option value="=">设置(=)</option></>}
                                                                            </select>
                                                                        </div>
                                                                        <div className="w-2/3">
                                                                            <label className="text-[10px] font-bold text-slate-500 block mb-1">值 (Value)</label>
                                                                            <input type="text" className="w-full border rounded px-2 py-1.5 text-xs" placeholder="值..." value={selectedNode.data.value || ''} onChange={(e) => updateNodeData('value', e.target.value)} />
                                                                        </div>
                                                                    </div>
                                                                )}
                                                            </div>
                                                        </div>
                                                    </div>
                                                )}
                                            </div>
                                        ) : (
                                            <div className="flex flex-col items-center justify-center h-full text-gray-400 mt-20">
                                                <Icons.Activity size={48} className="mb-3 opacity-20" />
                                                <p className="text-sm font-medium">在画布中选中节点以编辑</p>
                                            </div>
                                        )}
                                    </div>
                                </div>

                                {/* Blackboard Bottom Panel */}
                                <div className="bg-slate-50 border-t flex flex-col relative shadow-[0_-4px_10px_rgba(0,0,0,0.02)]" style={{ height: bottomPanelHeight }}>
                                    <div className="resizer-row" onMouseDown={(e) => { e.preventDefault(); setIsResizingBottom(true); }}></div>
                                    <div className="p-2 border-b bg-slate-100 flex items-center justify-between shadow-sm z-10">
                                        <div className="font-bold text-xs text-slate-700 flex items-center gap-1.5"><Icons.Database size={12} className="text-blue-500"/> 全局黑板 (Blackboard)</div>
                                        <button onClick={() => setBlackboard(p => [...p, { key: `var_${p.length}`, value: '0', type: 'float' }])} className="text-[10px] flex items-center gap-1 bg-white border border-slate-300 px-2 py-1 rounded shadow-sm hover:bg-blue-50 hover:border-blue-300 text-blue-600 font-bold transition"><Icons.Plus size={10} /> 添加变量</button>
                                    </div>
                                    <div className="flex-1 overflow-y-auto p-1 relative">
                                        <table className="w-full text-xs text-left border-collapse">
                                            <thead className="sticky top-0 bg-slate-50 shadow-sm z-10">
                                                <tr className="text-slate-500"><th className="py-1.5 pl-2 font-semibold">Key</th><th className="py-1.5 font-semibold w-20">Type</th><th className="py-1.5 font-semibold">Value</th><th className="py-1.5 w-8"></th></tr>
                                            </thead>
                                            <tbody>
                                                {blackboard.map((item, idx) => (
                                                    <tr key={idx} className="border-b border-slate-100 last:border-0 hover:bg-white transition-colors group">
                                                        <td className="p-1"><input type="text" className="w-full bg-transparent border-b border-transparent focus:border-blue-400 outline-none px-1 font-medium text-slate-700" value={item.key} onChange={(e) => { const n=[...blackboard]; n[idx].key=e.target.value; setBlackboard(n); }} /></td>
                                                        <td className="p-1"><select className="w-full bg-transparent text-slate-500 outline-none" value={item.type} onChange={(e) => { const n=[...blackboard]; n[idx].type=e.target.value; setBlackboard(n); }}><option value="int">Int</option><option value="float">Float</option><option value="string">String</option><option value="boolean">Bool</option><option value="enum">Enum</option></select></td>
                                                        <td className="p-1">
                                                            {item.type==='boolean' ? <select className="w-full bg-transparent text-blue-600 font-bold outline-none" value={item.value} onChange={(e)=>{ const n=[...blackboard]; n[idx].value=e.target.value; setBlackboard(n); }}><option value="true">true</option><option value="false">false</option></select> 
                                                            : <input type="text" className="w-full bg-transparent border-b border-transparent focus:border-blue-400 outline-none px-1 font-mono text-blue-600 font-bold" value={item.value} onChange={(e) => { const n=[...blackboard]; n[idx].value=e.target.value; setBlackboard(n); }} />}
                                                        </td>
                                                        <td className="p-1 text-center">
                                                            <button onClick={() => setBlackboard(p=>p.filter((_,i)=>i!==idx))} className="text-transparent group-hover:text-red-400 hover:!text-red-600 transition-colors p-1"><Icons.Trash2 size={12} /></button>
                                                        </td>
                                                    </tr>
                                                ))}
                                            </tbody>
                                        </table>
                                    </div>
                                </div>
                            </div>
                        )}
                    </div>
                </div>
            );
        }

        const root = ReactDOM.createRoot(document.getElementById('root'));
        root.render(<BehaviorTreeEditor />);
