        const { useState, useCallback, useRef, useEffect } = React;

        // --- SVG Icons ---
        const Icon = ({ path, size = 16, className, ...props }) => (<svg xmlns="http://www.w3.org/2000/svg" width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className={className} {...props}>{path}</svg>);
        const Icons = {
            ArrowRight: (p) => <Icon {...p} path={<><line x1="5" y1="12" x2="19" y2="12"/><polyline points="12 5 19 12 12 19"/></>} />,
            HelpCircle: (p) => <Icon {...p} path={<><circle cx="12" cy="12" r="10"/><path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"/><line x1="12" y1="17" x2="12.01" y2="17"/></>} />,
            RefreshCw: (p) => <Icon {...p} path={<><path d="M23 4v6h-6"/><path d="M1 20v-6h6"/><path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15"/></>} />,
            CheckCircle: (p) => <Icon {...p} path={<><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></>} />,
            AlertCircle: (p) => <Icon {...p} path={<><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></>} />,
            Activity: (p) => <Icon {...p} path={<polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/>} />,
            Upload: (p) => <Icon {...p} path={<><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></>} />,
            Download: (p) => <Icon {...p} path={<><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/></>} />,
            Trash2: (p) => <Icon {...p} path={<><polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/><line x1="10" y1="11" x2="10" y2="17"/><line x1="14" y1="11" x2="14" y2="17"/></>} />,
            Plus: (p) => <Icon {...p} path={<><line x1="12" y1="5" x2="12" y2="19"/><line x1="5" y1="12" x2="19" y2="12"/></>} />,
            Minus: (p) => <Icon {...p} path={<line x1="5" y1="12" x2="19" y2="12"/>} />,
            X: (p) => <Icon {...p} path={<><line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/></>} />,
            Shield: (p) => <Icon {...p} path={<><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></>} />,
            Crosshair: (p) => <Icon {...p} path={<><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></>} />,
            Zap: (p) => <Icon {...p} path={<polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/>} />,
            Play: (p) => <Icon {...p} path={<polygon points="5 3 19 12 5 21 5 3"/>} />,
            Square: (p) => <Icon {...p} path={<rect x="3" y="3" width="18" height="18" rx="2" ry="2"/>} />,
            Database: (p) => <Icon {...p} path={<><ellipse cx="12" cy="5" rx="9" ry="3"/><path d="M21 12c0 1.66-4 3-9 3s-9-1.34-9-3"/><path d="M3 5v14c0 1.66 4 3 9 3s9-1.34 9-3V5"/></>} />,
            Wifi: (p) => <Icon {...p} path={<path d="M5 12.55a11 11 0 0 1 14.08 0M1.42 9a16 16 0 0 1 21.16 0M8.53 16.11a6 6 0 0 1 6.95 0M12 20h.01"/>} />,
            WifiOff: (p) => <Icon {...p} path={<><line x1="1" y1="1" x2="23" y2="23"/><path d="M16.72 11.06A10.94 10.94 0 0 1 19 12.55"/><path d="M5 12.55a10.94 10.94 0 0 1 5.17-2.39"/><path d="M10.71 5.05A16 16 0 0 1 22.58 9"/><path d="M1.42 9a15.91 15.91 0 0 1 4.7-2.88"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><line x1="12" y1="20" x2="12.01" y2="20"/></>} />,
            FileCode: (p) => <Icon {...p} path={<><path d="M14.5 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V7.5L14.5 2z"/><polyline points="14 2 14 8 20 8"/><path d="m10 13-2 2 2 2"/><path d="m14 17 2-2-2-2"/></>} />,
            Settings: (p) => <Icon {...p} path={<><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"/></>} />,
            Save: (p) => <Icon {...p} path={<><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><polyline points="17 21 17 13 7 13 7 21"/><polyline points="7 3 7 8 15 8"/></>} />,
            Star: (p) => <Icon {...p} path={<polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/>} />,
            PanelLeft: (p) => <Icon {...p} path={<><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><line x1="9" y1="3" x2="9" y2="21"/></>} />,
            PanelRight: (p) => <Icon {...p} path={<><rect x="3" y="3" width="18" height="18" rx="2" ry="2"/><line x1="15" y1="3" x2="15" y2="21"/></>} />
        };

        // --- Base Node Definitions Library ---
        const BASE_NODE_TYPES = {
            // Standard BTCPP Nodes
            Sequence: { label: 'Sequence', group: 'std', baseType: 'SEQUENCE', color: '#f59e0b', icon: 'ArrowRight', desc: '按序执行子节点' },
            ReactiveSequence: { label: 'ReactiveSequence', group: 'std', baseType: 'SEQUENCE', color: '#f59e0b', icon: 'RefreshCw', desc: '响应式按序执行，遇假即断' },
            Fallback: { label: 'Fallback', group: 'std', baseType: 'FALLBACK', color: '#8b5cf6', icon: 'HelpCircle', desc: '依次尝试直到成功' },
            ReactiveFallback: { label: 'ReactiveFallback', group: 'std', baseType: 'FALLBACK', color: '#8b5cf6', icon: 'RefreshCw', desc: '响应式尝试，高优抢占' },
            ForceSuccess: { label: 'ForceSuccess', group: 'std', baseType: 'DECORATOR', color: '#ec4899', icon: 'CheckCircle', desc: '无论如何返回成功(Decorator)' },
            AlwaysSuccess: { label: 'AlwaysSuccess', group: 'std', baseType: 'ACTION', color: '#ec4899', icon: 'CheckCircle', desc: '永远成功(Action)' },

            // RM Nav Actions
            NavigateToPoseAction: { label: 'NavigateToPose', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '下发底盘导航目标' },
            SetCoordinate: { label: 'SetCoordinate', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '设置预置固定点' },
            SetTargetCoordinate: { label: 'SetTargetCoord', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '设置追击点(含限频)' },
            SelectPatrolPoint: { label: 'SelectPatrol', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '从数组选择巡逻点' },
            DirectVelocityControl: { label: 'VelocityControl', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '直接下发 cmd_vel 速度' },
            SetStairsPosition: { label: 'SetStairsPos', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '设置台阶预备点' },
            Wait: { label: 'Wait', group: 'rm_act', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '阻塞等待指定毫秒' },
            ChangeStance: { label: 'ChangeStance', group: 'rm_act', baseType: 'ACTION', color: '#ef4444', icon: 'Shield', desc: '下发云台底盘姿态(含CD)' },

            // RM Conditions
            CheckRetreatCondition: { label: 'CheckRetreat', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'AlertCircle', desc: '大掉血回家判断' },
            CheckInStairsZone: { label: 'CheckStairsZone', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'AlertCircle', desc: '检查是否处于台阶区' },
            CheckOutpostRemained: { label: 'CheckOutpost', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'AlertCircle', desc: '检查前哨站是否健在' },
            CheckTargetLocked: { label: 'CheckTarget', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'Crosshair', desc: '视觉锁敌判断(含防抖)' },
            CheckAttackStanceCondition: { label: 'CheckAttackStanceCondition', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'Crosshair', desc: '攻击姿态判定(含热量与前哨站)' },
            CheckMoveStanceCondition: { label: 'CheckMoveStanceCondition', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'Activity', desc: '移动姿态严格准入判定' },
            CheckDefendStanceCondition: { label: 'CheckDefendStanceCondition', group: 'rm_cond', baseType: 'CONDITION', color: '#10b981', icon: 'Shield', desc: '防御姿态判定(含追踪归防御与兜底)' },
            
            // Generic fallback
            Action: { label: 'Action', group: 'generic', baseType: 'ACTION', color: '#3b82f6', icon: 'Activity', desc: '通用动作' },
            Condition: { label: 'Condition', group: 'generic', baseType: 'CONDITION', color: '#10b981', icon: 'AlertCircle', desc: '通用条件' }
        };

        const INITIAL_BLACKBOARD = [
            { key: 'health', value: 400, type: 'float' },
            { key: 'enemy_outpost_destroyed', value: false, type: 'boolean' },
            { key: 'target_valid', value: false, type: 'boolean' },
            { key: 'nav_goal', value: '0,0', type: 'string' },
            { key: 'current_mode', value: 'PATROL', type: 'enum', enumName: 'NavMode', options: ['PATROL', 'ATTACK', 'RETREAT', 'RESPONSE', 'TRACING'] },
            { key: 'current_stance', value: 'MOVE', type: 'enum', enumName: 'SentryStance', options: ['MOVE', 'ATTACK', 'DEFEND', 'UNKNOWN'] }
        ];

        const NODE_WIDTH = 180;
        const NODE_HEIGHT = 70;
        const CANVAS_OFFSET = 5000;

        const generateId = (prefix) => `${prefix}-${Math.random().toString(36).substr(2, 9)}`;
        const downloadFile = (filename, content) => {
            const element = document.createElement('a');
            element.href = URL.createObjectURL(new Blob([content], {type: 'text/plain'}));
            element.download = filename;
            document.body.appendChild(element);
            element.click();
            document.body.removeChild(element);
        };

