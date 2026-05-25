"""
UAV Rocket-Tracking Intercept Mission Ground Station
Author: Lead Ground Station Software Architect & UI/UX Specialist
Description: Self-contained HTTP server serving a high-tech glassmorphism dashboard
             dedicated to visualizing rocket intercept flight profiles and 180s multi-physics telemetry.
"""

import json
import os
import sys
import webbrowser
from http.server import SimpleHTTPRequestHandler, HTTPServer

# Import our custom simulation modules
from propulsion_engine import BatteryConfig, ESCConfig, MotorConfig, PropellerConfig, PropulsionSystem
from flight_controller import load_rocket_profile, generate_optimal_drone_path

# Inline HTML, CSS, and JS for the Ground Station UI
HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Aura 7" Intercept Ground Station</title>
    <!-- Google Fonts: Outfit -->
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <!-- Plotly.js CDN -->
    <script src="https://cdn.plot.ly/plotly-2.24.1.min.js"></script>
    <style>
        :root {
            --bg-color: #030712;
            --card-bg: rgba(17, 24, 39, 0.45);
            --border-color: rgba(6, 182, 212, 0.15);
            --text-main: #f3f4f6;
            --text-muted: #9ca3af;
            --accent-cyan: #06b6d4;
            --accent-crimson: #ef4444;
            --accent-amber: #f59e0b;
            --accent-purple: #8b5cf6;
            --accent-green: #10b981;
            --accent-blue: #60a5fa;
            --glow-cyan: rgba(6, 182, 212, 0.15);
            --glow-red: rgba(239, 68, 68, 0.35);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: 'Outfit', sans-serif;
            -webkit-font-smoothing: antialiased;
        }

        body {
            background-color: var(--bg-color);
            color: var(--text-main);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            overflow-x: hidden;
            background-image: 
                radial-gradient(at 10% 20%, rgba(6, 182, 212, 0.05) 0px, transparent 50%),
                radial-gradient(at 90% 80%, rgba(239, 68, 68, 0.03) 0px, transparent 50%);
        }

        /* Ground Station Frame Layout */
        .ground-station {
            display: flex;
            min-height: 100vh;
            width: 100vw;
        }

        /* Sidebar Style */
        aside.sidebar {
            width: 260px;
            background: rgba(17, 24, 39, 0.7);
            backdrop-filter: blur(20px);
            border-right: 1px solid var(--border-color);
            display: flex;
            flex-direction: column;
            padding: 1.5rem;
            flex-shrink: 0;
        }

        .logo-section {
            margin-bottom: 2rem;
            text-align: center;
        }

        .logo-section h1 {
            font-size: 1.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--accent-cyan) 0%, #00f2fe 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            letter-spacing: 1px;
        }

        .logo-section p {
            font-size: 0.75rem;
            color: var(--text-muted);
            margin-top: 4px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .sidebar-section {
            margin-bottom: 2rem;
        }

        .sidebar-section h3 {
            font-size: 0.8rem;
            color: var(--accent-cyan);
            letter-spacing: 1px;
            margin-bottom: 1rem;
            text-transform: uppercase;
            border-bottom: 1px solid rgba(6, 182, 212, 0.1);
            padding-bottom: 4px;
        }

        .hardware-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: rgba(31, 41, 55, 0.35);
            padding: 10px 14px;
            border-radius: 8px;
            border: 1px solid rgba(255, 255, 255, 0.05);
            margin-bottom: 8px;
        }

        .hardware-item span {
            font-size: 0.85rem;
            color: var(--text-muted);
        }

        .hardware-item .val {
            font-weight: 600;
            color: var(--text-main);
        }

        .sidebar-footer {
            margin-top: auto;
            padding-top: 1rem;
        }

        /* Pulsing Red Command Button */
        @keyframes pulse-red {
            0% {
                box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7);
            }
            70% {
                box-shadow: 0 0 0 12px rgba(239, 68, 68, 0);
            }
            100% {
                box-shadow: 0 0 0 0 rgba(239, 68, 68, 0);
            }
        }

        .btn-pulse {
            width: 100%;
            background: linear-gradient(135deg, var(--accent-crimson) 0%, #b91c1c 100%);
            color: white;
            border: none;
            border-radius: 8px;
            padding: 14px 20px;
            font-size: 0.85rem;
            font-weight: 700;
            letter-spacing: 0.5px;
            cursor: pointer;
            box-shadow: 0 0 15px var(--glow-red);
            transition: all 0.3s ease;
            animation: pulse-red 2s infinite;
            text-align: center;
            line-height: 1.4;
        }

        .btn-pulse:hover {
            transform: translateY(-2px);
            box-shadow: 0 0 25px rgba(239, 68, 68, 0.6);
        }

        /* Main Panel Layout */
        main.main-content {
            flex-grow: 1;
            padding: 1.5rem;
            display: flex;
            flex-direction: column;
            overflow-y: auto;
            gap: 1.5rem;
        }

        .workspace {
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 25px;
            width: calc(100% - 280px);
            padding: 15px 25px;
            overflow-y: auto;
        }

        /* Header Style */
        header.header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: var(--card-bg);
            backdrop-filter: blur(30px);
            border: 1px solid var(--border-color);
            padding: 1.25rem 2rem;
            border-radius: 12px;
            gap: 2rem;
        }

        .title-section h1 {
            font-size: 1.4rem;
            font-weight: 700;
            letter-spacing: 0.5px;
            color: var(--text-main);
        }

        .title-section p {
            font-size: 0.75rem;
            color: var(--text-muted);
            margin-top: 2px;
        }

        /* KPI Matrix */
        .kpi-matrix {
            display: flex;
            gap: 1.5rem;
            flex-grow: 1;
            justify-content: flex-end;
        }

        .kpi-card {
            background: rgba(31, 41, 55, 0.4);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            padding: 8px 16px;
            display: flex;
            flex-direction: column;
            align-items: flex-end;
            min-width: 130px;
        }

        .kpi-card .label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .kpi-card .value {
            font-size: 1.35rem;
            font-weight: 700;
            color: var(--accent-cyan);
            margin: 2px 0;
        }

        .kpi-card .unit {
            font-size: 0.6rem;
            color: var(--text-muted);
        }

        /* Grid and Stack Layouts */
        .top-grid {
            display: grid;
            grid-template-columns: 1fr 1.3fr;
            gap: 25px;
        }

        .bottom-stack {
            display: flex;
            flex-direction: column;
            gap: 25px;
        }

        .bottom-stack .chart-card {
            min-height: 480px;
        }

        .chart-card {
            background: rgba(13, 19, 28, 0.55);
            backdrop-filter: blur(30px);
            border: 1px solid rgba(69, 243, 255, 0.18);
            border-radius: 12px;
            padding: 1.25rem;
            display: flex;
            flex-direction: column;
            height: 100%;
            box-shadow: 0 15px 35px rgba(0, 0, 0, 0.6);
        }

        .chart-card h2 {
            font-size: 0.85rem;
            font-weight: 600;
            letter-spacing: 1px;
            color: var(--text-main);
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            padding-bottom: 8px;
            margin-bottom: 12px;
        }

        .chart-container {
            flex-grow: 1;
            width: 100%;
            height: 100%;
            position: relative;
        }

        /* Loading Screen overlay */
        #loading {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(3, 7, 18, 0.85);
            backdrop-filter: blur(8px);
            z-index: 9999;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            opacity: 0;
            pointer-events: none;
            transition: opacity 0.3s ease;
        }

        #loading.active {
            opacity: 1;
            pointer-events: auto;
        }

        .spinner {
            width: 50px;
            height: 50px;
            border: 3px solid rgba(6, 182, 212, 0.1);
            border-radius: 50%;
            border-top-color: var(--accent-cyan);
            animation: spin 1s ease-in-out infinite;
            margin-bottom: 1rem;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        #loading-text {
            font-size: 0.9rem;
            color: var(--text-main);
            font-weight: 500;
            letter-spacing: 0.5px;
        }

        /* Responsive styling */
        @media (max-width: 1024px) {
            .ground-station {
                flex-direction: column;
            }
            aside.sidebar {
                width: 100%;
                border-right: none;
                border-bottom: 1px solid var(--border-color);
            }
            .dashboard-grid {
                grid-template-columns: 1fr;
                min-height: auto;
            }
        }

        /* High-Tech Telemetry HUD Panel */
        .hud-telemetry-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(105px, 1fr));
            gap: 0.75rem;
            margin-bottom: 1.25rem;
            padding: 0.5rem;
            background: rgba(17, 24, 39, 0.4);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            backdrop-filter: blur(10px);
            box-shadow: 0 4px 30px rgba(0, 0, 0, 0.3);
        }

        .hud-card {
            display: flex;
            flex-direction: column;
            padding: 0.5rem 0.75rem;
            background: rgba(31, 41, 55, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 6px;
            position: relative;
            transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .hud-card.alert {
            border-color: rgba(249, 115, 22, 0.6) !important;
            background: rgba(249, 115, 22, 0.08);
            box-shadow: 0 0 10px rgba(249, 115, 22, 0.15);
        }

        .hud-card .hud-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 2px;
        }

        .hud-card .hud-value {
            font-family: 'Courier New', Courier, monospace;
            font-size: 1.15rem;
            font-weight: 700;
            color: var(--text-main);
        }

        .hud-card .hud-unit {
            font-size: 0.6rem;
            color: var(--text-muted);
            margin-top: 1px;
        }

        /* Border highlights for matching plot trace colors */
        #hud-time { border-left: 3px solid var(--accent-cyan); }
        #hud-tracking-error { border-left: 3px solid #ef4444; }
        #hud-throttle { border-left: 3px solid #f59e0b; }
        #hud-soc { border-left: 3px solid #10b981; }
        #hud-temp { border-left: 3px solid #f87171; }
        #hud-voltage { border-left: 3px solid #22d3ee; }
        #hud-load { border-left: 3px solid #c084fc; }
        #hud-gimbal { border-left: 3px solid #60a5fa; }

        /* Status indicator dots */
        .hud-indicator {
            position: absolute;
            top: 8px;
            right: 8px;
            width: 6px;
            height: 6px;
            border-radius: 50%;
            transition: all 0.2s ease;
        }

        .hud-indicator.safe {
            background-color: var(--accent-green);
            box-shadow: 0 0 6px var(--accent-green);
        }

        .hud-indicator.warning {
            background-color: #f97316; /* Neon Orange */
            box-shadow: 0 0 10px #f97316, 0 0 20px #f97316;
            animation: pulse-orange 1s infinite alternate;
        }

        @keyframes pulse-orange {
            0% { opacity: 0.4; transform: scale(0.9); }
            100% { opacity: 1; transform: scale(1.2); }
        }

        /* Tab switching buttons style */
        .tab-navigation {
            display: flex;
            gap: 15px;
            margin-bottom: 5px;
        }

        .tab-btn {
            background: rgba(31, 41, 55, 0.45);
            border: 1px solid var(--border-color);
            color: var(--text-muted);
            padding: 10px 20px;
            border-radius: 8px;
            cursor: pointer;
            font-size: 0.85rem;
            font-weight: 600;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            letter-spacing: 0.5px;
        }

        .tab-btn:hover {
            color: var(--text-main);
            background: rgba(6, 182, 212, 0.1);
            border-color: rgba(6, 182, 212, 0.35);
        }

        .tab-btn.active {
            color: var(--accent-cyan);
            border-color: var(--accent-cyan);
            background: rgba(6, 182, 212, 0.15);
            box-shadow: 0 0 15px rgba(6, 182, 212, 0.2);
        }

        .tab-content {
            transition: opacity 0.25s ease-in-out;
        }

        /* Architect Sub-systems parameter grid */
        .architect-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
            gap: 20px;
            width: 100%;
        }

        .architect-card {
            background: rgba(13, 19, 28, 0.55);
            backdrop-filter: blur(30px);
            border: 1px solid rgba(69, 243, 255, 0.18);
            border-radius: 12px;
            padding: 1.25rem;
            display: flex;
            flex-direction: column;
            box-shadow: 0 15px 35px rgba(0, 0, 0, 0.6);
            transition: all 0.3s ease;
        }

        .architect-card:hover {
            border-color: rgba(6, 182, 212, 0.35);
            box-shadow: 0 15px 35px rgba(6, 182, 212, 0.1);
        }

        .architect-card h3 {
            font-size: 0.85rem;
            font-weight: 600;
            letter-spacing: 1px;
            color: var(--text-main);
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            padding-bottom: 8px;
            margin-bottom: 12px;
            text-transform: uppercase;
        }

        .input-row {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .input-group {
            display: flex;
            align-items: center;
            justify-content: space-between;
            background: rgba(31, 41, 55, 0.35);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            padding: 8px 12px;
            transition: all 0.25s ease;
        }

        .input-group:focus-within {
            border-color: var(--accent-cyan);
            background: rgba(6, 182, 212, 0.05);
            box-shadow: 0 0 8px rgba(6, 182, 212, 0.1);
        }

        .input-group label {
            font-size: 0.8rem;
            color: var(--text-muted);
            flex: 1;
            margin-right: 10px;
        }

        .input-group input {
            background: transparent;
            border: none;
            color: var(--text-main);
            font-family: 'Courier New', Courier, monospace;
            font-size: 0.85rem;
            font-weight: bold;
            text-align: right;
            width: 80px;
            outline: none;
        }

        /* Unit labels styling */
        .input-group .unit {
            font-size: 0.75rem;
            color: var(--text-muted);
            width: 45px;
            text-align: left;
            margin-left: 8px;
            border-left: 1px solid rgba(255, 255, 255, 0.08);
            padding-left: 8px;
        }

        /* System Overview Monitor Board (系統概要監控板) */
        .system-specs-summary {
            background: rgba(13, 19, 28, 0.65);
            border: 1px solid rgba(69, 243, 255, 0.22);
            backdrop-filter: blur(25px);
            border-radius: 8px;
            padding: 12px 25px;
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 20px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.5);
        }

        .spec-block {
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 12px 8px;
            background: rgba(31, 41, 55, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 8px;
            position: relative;
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }

        .spec-block:hover {
            border-color: rgba(6, 182, 212, 0.3);
            background: rgba(6, 182, 212, 0.05);
            box-shadow: 0 0 15px rgba(6, 182, 212, 0.08);
        }

        .spec-block .spec-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.8px;
            margin-bottom: 6px;
            text-align: center;
            line-height: 1.3;
        }

        .spec-block .spec-value {
            font-family: 'Courier New', Courier, monospace;
            font-size: 1.55rem;
            font-weight: 700;
            line-height: 1.2;
        }

        .spec-block .spec-unit {
            font-size: 0.6rem;
            color: var(--text-muted);
            margin-top: 4px;
            letter-spacing: 0.5px;
            text-align: center;
        }

        .spec-block:nth-child(1) .spec-value { color: #06b6d4; text-shadow: 0 0 15px rgba(6, 182, 212, 0.4); }
        .spec-block:nth-child(2) .spec-value { color: #f59e0b; text-shadow: 0 0 15px rgba(245, 158, 11, 0.4); }
        .spec-block:nth-child(3) .spec-value { color: #10b981; text-shadow: 0 0 15px rgba(16, 185, 129, 0.4); }
        .spec-block:nth-child(4) .spec-value { color: #8b5cf6; text-shadow: 0 0 15px rgba(139, 92, 246, 0.4); }

        @media (max-width: 1024px) {
            .system-specs-summary {
                grid-template-columns: repeat(2, 1fr);
            }
        }
    </style>
</head>
<body>
    <!-- Main Frame -->
    <div class="ground-station">
        <!-- Sidebar -->
        <aside class="sidebar">
            <div class="logo-section">
                <h1>AURA 7" INTERCEPT</h1>
                <p>Telemetry Ground Station</p>
            </div>

            <!-- Hardware Constraints Section -->
            <div class="sidebar-section">
                <h3>Hardware Parameters</h3>
                
                <div class="hardware-item">
                    <span>Frame</span>
                    <span class="val">Foxeer Aura 7"</span>
                </div>
                
                <div class="hardware-item">
                    <span>Motor</span>
                    <span class="val">Xnova 2812 1300KV</span>
                </div>
                
                <div class="hardware-item">
                    <span>Propeller</span>
                    <span class="val">Gemfan 7050</span>
                </div>

                <div class="hardware-item">
                    <span>Battery</span>
                    <span class="val">6S 6500mAh</span>
                </div>
            </div>

            <!-- Mission Control Button -->
            <div class="sidebar-footer">
                <button class="btn-pulse" onclick="runSimulation()">
                    🎯 GENERATE INTERCEPT TRAJECTORY & ANALYZE PROFILE
                </button>
            </div>
        </aside>

        <!-- Main Panel -->
        <main class="main-content workspace">
            <!-- Header & KPI matrix -->
            <header class="header">
                <div class="title-section">
                    <h1>MISSION STATUS MONITOR</h1>
                    <p>Dynamic Intercept & Tracking Dashboard</p>
                </div>
                
                <div class="kpi-matrix">
                    <div class="kpi-card" id="kpi-gforce">
                        <span class="label">Max Drone Acceleration</span>
                        <span class="value" id="val-gforce">-</span>
                        <span class="unit">G</span>
                    </div>
                    <div class="kpi-card" id="kpi-temp">
                        <span class="label">Peak Motor Temp Rise</span>
                        <span class="value" id="val-temp">-</span>
                        <span class="unit">°C rise</span>
                    </div>
                    <div class="kpi-card" id="kpi-soc">
                        <span class="label">Final Remaining SOC</span>
                        <span class="value" id="val-soc">-</span>
                        <span class="unit">%</span>
                    </div>
                    <div class="kpi-card" id="kpi-error">
                        <span class="label">Max Tracking Error</span>
                        <span class="value" id="val-error">-</span>
                        <span class="unit">meters</span>
                    </div>
                </div>
            </header>

            <!-- Tab Navigation Bar -->
            <div class="tab-navigation" style="margin-bottom: 15px;">
                <button class="tab-btn active" onclick="switchTab('tab-telemetry', this)">
                    📊 Flight Telemetry & Mission Control
                </button>
                <button class="tab-btn" onclick="switchTab('tab-architect', this)">
                    🛠️ UAV Component Architect
                </button>
            </div>

            <!-- System Overview Monitor Board (系統概要監控板) -->
            <div class="system-specs-summary">
                <div class="spec-block">
                    <span class="spec-label">Total Aircraft Mass 全機總重</span>
                    <span class="spec-value" id="spec-total-mass">--</span>
                    <span class="spec-unit">kg (AUW incl. 15% margin)</span>
                </div>
                <div class="spec-block">
                    <span class="spec-label">Max Aggregate Thrust 總最大推力</span>
                    <span class="spec-value" id="spec-max-thrust">--</span>
                    <span class="spec-unit">kg (4-motor combined)</span>
                </div>
                <div class="spec-block">
                    <span class="spec-label">Thrust-to-Weight Ratio 推重比</span>
                    <span class="spec-value" id="spec-twr">--</span>
                    <span class="spec-unit">TWR ratio</span>
                </div>
                <div class="spec-block">
                    <span class="spec-label">Est. Hover Endurance 預估懸停時長</span>
                    <span class="spec-value" id="spec-hover-time">--</span>
                    <span class="spec-unit">minutes (steady-state)</span>
                </div>
            </div>

            <!-- Tab 1: Telemetry Dashboard -->
            <div id="tab-telemetry" class="tab-content" style="display: flex; flex-direction: column; gap: 25px;">
                <!-- Top Grid (Asymmetric side-by-side plots) -->
                <div class="top-grid">
                    <!-- Static Propulsion Curves Card -->
                    <div class="chart-card" style="height: 380px;">
                        <h2>📊 STATIC PROPULSION CURVES (100% SOC)</h2>
                        <div class="chart-container" id="plot-static"></div>
                    </div>
                    <!-- 3D Trajectory Card -->
                    <div class="chart-card" style="height: 380px;">
                        <h2>📍 3D INTERCEPT TRAJECTORY OVERLAY</h2>
                        <div class="chart-container" id="plot-3d"></div>
                    </div>
                </div>

                <!-- Bottom Stack (3-Panel Telemetry) -->
                <div class="bottom-stack">
                    <!-- Chart A: Kinematics -->
                    <div class="chart-card">
                        <h2>📊 CHART A: SPATIAL KINEMATICS</h2>
                        <div class="hud-telemetry-grid">
                            <div class="hud-card" id="hud-card-time-a" style="border-left: 3px solid var(--accent-cyan);">
                                <span class="hud-label">Time</span>
                                <span class="hud-value" id="hud-a-time">-</span>
                                <span class="hud-unit">s</span>
                            </div>
                            <div class="hud-card" id="hud-card-vel" style="border-left: 3px solid #ef4444;">
                                <span class="hud-indicator" id="ind-vel"></span>
                                <span class="hud-label">Velocity</span>
                                <span class="hud-value" id="hud-a-vel">-</span>
                                <span class="hud-unit">m/s</span>
                            </div>
                            <div class="hud-card" id="hud-card-gforce" style="border-left: 3px solid #c084fc;">
                                <span class="hud-indicator" id="ind-gforce"></span>
                                <span class="hud-label">Acceleration</span>
                                <span class="hud-value" id="hud-a-gforce">-</span>
                                <span class="hud-unit">G</span>
                            </div>
                            <div class="hud-card" id="hud-card-hspan" style="border-left: 3px solid #f59e0b;">
                                <span class="hud-label">Horizontal Span (E)</span>
                                <span class="hud-value" id="hud-a-hspan">-</span>
                                <span class="hud-unit">m</span>
                            </div>
                            <div class="hud-card" id="hud-card-lspan" style="border-left: 3px solid #10b981;">
                                <span class="hud-label">Longitudinal Span (N)</span>
                                <span class="hud-value" id="hud-a-lspan">-</span>
                                <span class="hud-unit">m</span>
                            </div>
                        </div>
                        <div class="chart-container" id="plot-kinematics"></div>
                    </div>

                    <!-- Chart B: Power -->
                    <div class="chart-card">
                        <h2>🔌 CHART B: ELECTRICAL ENERGY</h2>
                        <div class="hud-telemetry-grid">
                            <div class="hud-card" id="hud-card-time-b" style="border-left: 3px solid var(--accent-cyan);">
                                <span class="hud-label">Time</span>
                                <span class="hud-value" id="hud-b-time">-</span>
                                <span class="hud-unit">s</span>
                            </div>
                            <div class="hud-card" id="hud-card-vol" style="border-left: 3px solid #22d3ee;">
                                <span class="hud-indicator" id="ind-vol"></span>
                                <span class="hud-label">Voltage Sag</span>
                                <span class="hud-value" id="hud-b-vol">-</span>
                                <span class="hud-unit">V</span>
                            </div>
                            <div class="hud-card" id="hud-card-soc" style="border-left: 3px solid #10b981;">
                                <span class="hud-indicator" id="ind-soc"></span>
                                <span class="hud-label">Battery SOC</span>
                                <span class="hud-value" id="hud-b-soc">-</span>
                                <span class="hud-unit">%</span>
                            </div>
                        </div>
                        <div class="chart-container" id="plot-power"></div>
                    </div>

                    <!-- Chart C: Attitude -->
                    <div class="chart-card">
                        <h2>🔄 CHART C: ANGULAR DYNAMICS (ATTITUDE)</h2>
                        <div class="hud-telemetry-grid">
                            <div class="hud-card" id="hud-card-time-c" style="border-left: 3px solid var(--accent-cyan);">
                                <span class="hud-label">Time</span>
                                <span class="hud-value" id="hud-c-time">-</span>
                                <span class="hud-unit">s</span>
                            </div>
                            <div class="hud-card" id="hud-card-pitch" style="border-left: 3px solid #f87171;">
                                <span class="hud-label">Pitch</span>
                                <span class="hud-value" id="hud-c-pitch">-</span>
                                <span class="hud-unit">deg</span>
                            </div>
                            <div class="hud-card" id="hud-card-roll" style="border-left: 3px solid #60a5fa;">
                                <span class="hud-label">Roll</span>
                                <span class="hud-value" id="hud-c-roll">-</span>
                                <span class="hud-unit">deg</span>
                            </div>
                            <div class="hud-card" id="hud-card-yaw" style="border-left: 3px solid #c084fc;">
                                <span class="hud-label">Yaw</span>
                                <span class="hud-value" id="hud-c-yaw">-</span>
                                <span class="hud-unit">deg</span>
                            </div>
                        </div>
                        <div class="chart-container" id="plot-attitude"></div>
                    </div>
                </div>
            </div>

            <!-- Tab 2: UAV Component Architect -->
            <div id="tab-architect" class="tab-content" style="display: none;">
                <div class="architect-grid">
                    <!-- [A] CORE STRUCTURE -->
                    <div class="architect-card">
                        <h3>[A] Core Structure (核心組件)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-frame-weight">Frame Weight</label>
                                <input type="number" id="input-frame-weight" value="184.0" step="any">
                                <span class="unit">g</span>
                            </div>
                            <div class="input-group">
                                <label for="input-arm-length">Arm Length</label>
                                <input type="number" id="input-arm-length" value="280.0" step="any">
                                <span class="unit">mm</span>
                            </div>
                            <div class="input-group">
                                <label for="input-landing-gear-cd">Landing Gear Drag Cd</label>
                                <input type="number" id="input-landing-gear-cd" value="0.15" step="any">
                                <span class="unit">Cd</span>
                            </div>
                        </div>
                    </div>

                    <!-- [B] PROPULSION SYSTEM -->
                    <div class="architect-card">
                        <h3>[B] Propulsion System (動力系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-motor-kv">Motor KV Rating</label>
                                <input type="number" id="input-motor-kv" value="1300" step="any">
                                <span class="unit">RPM/V</span>
                            </div>
                            <div class="input-group">
                                <label for="input-motor-r">Motor Phase Resistance</label>
                                <input type="number" id="input-motor-r" value="0.021" step="0.001">
                                <span class="unit">Ω</span>
                            </div>
                            <div class="input-group">
                                <label for="input-esc-current">ESC Continuous Current</label>
                                <input type="number" id="input-esc-current" value="80.0" step="any">
                                <span class="unit">A</span>
                            </div>
                            <div class="input-group">
                                <label for="input-esc-r">ESC Internal Resistance</label>
                                <input type="number" id="input-esc-r" value="0.0015" step="0.0001">
                                <span class="unit">Ω</span>
                            </div>
                            <div class="input-group">
                                <label for="input-prop-diameter">Propeller Diameter</label>
                                <input type="number" id="input-prop-diameter" value="7.0" step="any">
                                <span class="unit">in</span>
                            </div>
                            <div class="input-group">
                                <label for="input-prop-pitch">Propeller Pitch</label>
                                <input type="number" id="input-prop-pitch" value="5.0" step="any">
                                <span class="unit">in</span>
                            </div>
                            <div class="input-group">
                                <label for="input-prop-blades">Propeller Blade Count</label>
                                <input type="number" id="input-prop-blades" value="3" step="1">
                                <span class="unit">blades</span>
                            </div>
                        </div>
                    </div>

                    <!-- [C] POWER SYSTEM -->
                    <div class="architect-card">
                        <h3>[C] Power System (電力系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-battery-capacity">Battery Total Capacity</label>
                                <input type="number" id="input-battery-capacity" value="6500" step="any">
                                <span class="unit">mAh</span>
                            </div>
                            <div class="input-group">
                                <label for="input-battery-cells">Battery Cell Count S</label>
                                <input type="number" id="input-battery-cells" value="6" step="1">
                                <span class="unit">cells</span>
                            </div>
                            <div class="input-group">
                                <label for="input-battery-r">Cell Internal Resistance</label>
                                <input type="number" id="input-battery-r" value="0.003" step="0.001">
                                <span class="unit">Ω</span>
                            </div>
                            <div class="input-group">
                                <label for="input-pdb-current">PDB Peak Current</label>
                                <input type="number" id="input-pdb-current" value="360.0" step="any">
                                <span class="unit">A</span>
                            </div>
                            <div class="input-group">
                                <label for="input-bec-voltage">BEC Regulated Output</label>
                                <input type="number" id="input-bec-voltage" value="5.0" step="any">
                                <span class="unit">V</span>
                            </div>
                        </div>
                    </div>

                    <!-- [D] CONTROL SYSTEM -->
                    <div class="architect-card">
                        <h3>[D] Control System (控制系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-fc-refresh">FC Loop Refresh Rate</label>
                                <input type="number" id="input-fc-refresh" value="8.0" step="any">
                                <span class="unit">kHz</span>
                            </div>
                            <div class="input-group">
                                <label for="input-imu-delay">IMU Sensor Filter Delay</label>
                                <input type="number" id="input-imu-delay" value="1.5" step="any">
                                <span class="unit">ms</span>
                            </div>
                            <div class="input-group">
                                <label for="input-gps-refresh">GPS Update Frequency</label>
                                <input type="number" id="input-gps-refresh" value="10.0" step="any">
                                <span class="unit">Hz</span>
                            </div>
                        </div>
                    </div>

                    <!-- [E] SENSING SYSTEM -->
                    <div class="architect-card">
                        <h3>[E] Sensing System (感測系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-lidar-weight">LiDAR Component Weight</label>
                                <input type="number" id="input-lidar-weight" value="45.0" step="any">
                                <span class="unit">g</span>
                            </div>
                            <div class="input-group">
                                <label for="input-vision-power">Vision AI Camera Power</label>
                                <input type="number" id="input-vision-power" value="3.5" step="any">
                                <span class="unit">W</span>
                            </div>
                            <div class="input-group">
                                <label for="input-tof-range">ToF Safe Obstacle Range</label>
                                <input type="number" id="input-tof-range" value="15.0" step="any">
                                <span class="unit">m</span>
                            </div>
                        </div>
                    </div>

                    <!-- [F] COMMUNICATION SYSTEM -->
                    <div class="architect-card">
                        <h3>[F] Communication System (通訊系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-rx-latency">RX Protocol Latency</label>
                                <input type="number" id="input-rx-latency" value="4.0" step="any">
                                <span class="unit">ms</span>
                            </div>
                            <div class="input-group">
                                <label for="input-telemetry-freq">Telemetry Frequency Band</label>
                                <input type="number" id="input-telemetry-freq" value="915.0" step="any">
                                <span class="unit">MHz</span>
                            </div>
                            <div class="input-group">
                                <label for="input-vtx-power">VTX Transmission Power</label>
                                <input type="number" id="input-vtx-power" value="800.0" step="any">
                                <span class="unit">mW</span>
                            </div>
                        </div>
                    </div>

                    <!-- [G] PAYLOAD SYSTEM -->
                    <div class="architect-card">
                        <h3>[G] Payload System (酬載系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-camera-weight">Camera Lens Mass Weight</label>
                                <input type="number" id="input-camera-weight" value="215.0" step="any">
                                <span class="unit">g</span>
                            </div>
                            <div class="input-group">
                                <label for="input-gimbal-axes">Gimbal Control Axis Count</label>
                                <input type="number" id="input-gimbal-axes" value="3" step="1">
                                <span class="unit">axes</span>
                            </div>
                        </div>
                    </div>

                    <!-- [H] GROUND SYSTEM -->
                    <div class="architect-card">
                        <h3>[H] Ground System (地面系統)</h3>
                        <div class="input-row">
                            <div class="input-group">
                                <label for="input-radio-range">Radio Maximum Link Range</label>
                                <input type="number" id="input-radio-range" value="15.0" step="any">
                                <span class="unit">km</span>
                            </div>
                            <div class="input-group">
                                <label for="input-gcs-latency">GCS Video Display Latency</label>
                                <input type="number" id="input-gcs-latency" value="28.0" step="any">
                                <span class="unit">ms</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </main>
    </div>

    <!-- Loading Screen -->
    <div id="loading">
        <div class="spinner"></div>
        <div id="loading-text">Loading trajectory data...</div>
    </div>

    <!-- Main Controller Logic Script -->
    <script>
        // Dynamic API Base detection for CORS/preflight scenarios
        const API_BASE = window.location.port === '8000' ? '' : 'http://127.0.0.1:8000';
        let currentSimData = null;

        function showLoading(text) {
            document.getElementById('loading-text').innerText = text;
            document.getElementById('loading').classList.add('active');
        }

        function hideLoading() {
            document.getElementById('loading').classList.remove('active');
        }

        function switchTab(tabId, btn) {
            // Deactivate all tabs
            document.querySelectorAll('.tab-content').forEach(tab => {
                tab.style.display = 'none';
            });
            document.querySelectorAll('.tab-btn').forEach(b => {
                b.classList.remove('active');
            });
            
            // Activate selected tab
            document.getElementById(tabId).style.display = tabId === 'tab-telemetry' ? 'flex' : 'block';
            btn.classList.add('active');
            
            // Trigger Plotly resizes
            setTimeout(() => {
                const plots = ['plot-static', 'plot-3d', 'plot-kinematics', 'plot-power', 'plot-attitude'];
                plots.forEach(id => {
                    const el = document.getElementById(id);
                    if (el) {
                        Plotly.Plots.resize(el);
                    }
                });
            }, 50);
        }

        function recalculateAircraftDesignSpecs() {
            // Harvest relevant inputs from Component Architect tab
            const frame_weight = parseFloat(document.getElementById('input-frame-weight').value) || 184.0;
            const arm_length = parseFloat(document.getElementById('input-arm-length').value) || 280.0;
            const motor_kv = parseFloat(document.getElementById('input-motor-kv').value) || 1300;
            const prop_diameter = parseFloat(document.getElementById('input-prop-diameter').value) || 7.0;
            const prop_pitch = parseFloat(document.getElementById('input-prop-pitch').value) || 5.0;
            const prop_blades = parseInt(document.getElementById('input-prop-blades').value) || 3;
            const battery_capacity = parseFloat(document.getElementById('input-battery-capacity').value) || 6500;
            const battery_cells = parseInt(document.getElementById('input-battery-cells').value) || 6;
            const lidar_weight = parseFloat(document.getElementById('input-lidar-weight').value) || 45.0;
            const camera_weight = parseFloat(document.getElementById('input-camera-weight').value) || 215.0;
            const gimbal_axes = parseInt(document.getElementById('input-gimbal-axes').value) || 3;

            // ── [1] Total Aircraft Mass (mirrors backend _handle_simulate exactly) ──
            const m_frame = frame_weight;
            const m_arms = 4.0 * 20.0 * (arm_length / 280.0);
            const m_motors = 4.0 * 80.0;
            const m_battery = battery_capacity * battery_cells * 0.012246;
            const m_lidar = lidar_weight;
            const m_camera = camera_weight;
            const m_gimbal = 120.0 * (gimbal_axes / 3.0);
            const m_landing_gear = 25.0;
            const m_esc = 45.0;
            const m_props = 4.0 * 8.9;
            const m_pdb = 12.0;
            const m_bec = 8.0;
            const m_fc = 10.0;
            const m_imu = 2.0;
            const m_gps = 15.0;
            const m_vision = 25.0;
            const m_tof = 5.0;
            const m_rx = 3.0;
            const m_telemetry = 10.0;
            const m_vtx = 15.0;

            const total_component_g = m_frame + m_arms + m_motors + m_battery + m_lidar + m_vtx +
                m_camera + m_gimbal + m_landing_gear + m_esc + m_props +
                m_pdb + m_bec + m_fc + m_imu + m_gps + m_vision + m_tof +
                m_rx + m_telemetry;
            const auw_g = total_component_g * 1.15;
            const mass_kg = auw_g / 1000.0;

            // ── [2] Maximum Aggregate Thrust ──
            // Calibrated anchor: 2860 g/motor at default 7x5x3, 1300KV, 6S
            // Scales with propeller theory: T ∝ D^3.5 × pitch^0.75 × blades^0.4 × (KV×V)^1.5
            const thrust_per_motor_g = 2860.0
                * Math.pow(prop_diameter / 7.0, 3.5)
                * Math.pow(prop_pitch / 5.0, 0.75)
                * Math.pow(prop_blades / 3, 0.4)
                * Math.pow(motor_kv / 1300, 1.5)
                * Math.pow(battery_cells / 6, 1.5);
            const total_thrust_g = 4.0 * thrust_per_motor_g;
            const thrust_kg = total_thrust_g / 1000.0;

            // ── [3] Thrust-to-Weight Ratio ──
            const twr = mass_kg > 0 ? thrust_kg / mass_kg : 0;

            // ── [4] Theoretical Hover Endurance ──
            // Momentum theory: P_hover = K × m^1.5 / D²
            // K calibrated so 1.90 kg / 6500 mAh / 6S / 7" → 13.1 min
            const K_HOVER = 7.977;
            const D_m = prop_diameter * 0.0254;
            const V_nom = battery_cells * 3.7;
            const P_hover = K_HOVER * Math.pow(mass_kg, 1.5) / (D_m * D_m);
            const I_hover = V_nom > 0 ? P_hover / V_nom : 999;
            const hover_min = I_hover > 0 ? (battery_capacity / 1000.0) / I_hover * 60.0 : 0;

            // ── Render to monitor board ──
            document.getElementById('spec-total-mass').innerText = mass_kg.toFixed(2);
            document.getElementById('spec-max-thrust').innerText = thrust_kg.toFixed(2);
            document.getElementById('spec-twr').innerText = twr.toFixed(2) + ' : 1';
            document.getElementById('spec-hover-time').innerText = hover_min.toFixed(1);
        }

        // Attach real-time recalculation listeners to every architect input
        function attachArchitectListeners() {
            const inputIds = [
                'input-frame-weight', 'input-arm-length', 'input-landing-gear-cd',
                'input-motor-kv', 'input-motor-r', 'input-esc-current', 'input-esc-r',
                'input-prop-diameter', 'input-prop-pitch', 'input-prop-blades',
                'input-battery-capacity', 'input-battery-cells', 'input-battery-r',
                'input-pdb-current', 'input-bec-voltage',
                'input-fc-refresh', 'input-imu-delay', 'input-gps-refresh',
                'input-lidar-weight', 'input-vision-power', 'input-tof-range',
                'input-rx-latency', 'input-telemetry-freq', 'input-vtx-power',
                'input-camera-weight', 'input-gimbal-axes',
                'input-radio-range', 'input-gcs-latency'
            ];
            inputIds.forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.addEventListener('input', recalculateAircraftDesignSpecs);
                }
            });
        }

        function gatherArchitectInputs() {
            return {
                frame_weight: parseFloat(document.getElementById('input-frame-weight').value) || 184.0,
                arm_length: parseFloat(document.getElementById('input-arm-length').value) || 280.0,
                landing_gear_cd: parseFloat(document.getElementById('input-landing-gear-cd').value) || 0.15,
                
                motor_kv: parseFloat(document.getElementById('input-motor-kv').value) || 1300,
                motor_r: parseFloat(document.getElementById('input-motor-r').value) || 0.021,
                esc_current: parseFloat(document.getElementById('input-esc-current').value) || 80.0,
                esc_r: parseFloat(document.getElementById('input-esc-r').value) || 0.0015,
                prop_diameter: parseFloat(document.getElementById('input-prop-diameter').value) || 7.0,
                prop_pitch: parseFloat(document.getElementById('input-prop-pitch').value) || 5.0,
                prop_blades: parseInt(document.getElementById('input-prop-blades').value) || 3,
                
                battery_capacity: parseFloat(document.getElementById('input-battery-capacity').value) || 6500,
                battery_cells: parseInt(document.getElementById('input-battery-cells').value) || 6,
                battery_r: parseFloat(document.getElementById('input-battery-r').value) || 0.003,
                pdb_current: parseFloat(document.getElementById('input-pdb-current').value) || 360.0,
                bec_voltage: parseFloat(document.getElementById('input-bec-voltage').value) || 5.0,
                
                fc_refresh: parseFloat(document.getElementById('input-fc-refresh').value) || 8.0,
                imu_delay: parseFloat(document.getElementById('input-imu-delay').value) || 1.5,
                gps_refresh: parseFloat(document.getElementById('input-gps-refresh').value) || 10.0,
                
                lidar_weight: parseFloat(document.getElementById('input-lidar-weight').value) || 45.0,
                vision_power: parseFloat(document.getElementById('input-vision-power').value) || 3.5,
                tof_range: parseFloat(document.getElementById('input-tof-range').value) || 15.0,
                
                rx_latency: parseFloat(document.getElementById('input-rx-latency').value) || 4.0,
                telemetry_freq: parseFloat(document.getElementById('input-telemetry-freq').value) || 915.0,
                vtx_power: parseFloat(document.getElementById('input-vtx-power').value) || 800.0,
                
                camera_weight: parseFloat(document.getElementById('input-camera-weight').value) || 215.0,
                gimbal_axes: parseInt(document.getElementById('input-gimbal-axes').value) || 3,
                
                radio_range: parseFloat(document.getElementById('input-radio-range').value) || 15.0,
                gcs_latency: parseFloat(document.getElementById('input-gcs-latency').value) || 28.0
            };
        }

        async function runSimulation() {
            showLoading("Designing trajectory & running multi-physics simulation (180s)...");
            
            try {
                const payload = gatherArchitectInputs();
                payload.maneuver_preset = 'rocket_tracking';
                
                const response = await fetch(API_BASE + '/api/simulate', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });
                
                const data = await response.json();
                
                if (data.status === 'success') {
                    currentSimData = data;
                    updateKPIs(data);
                    renderPlots(data);
                    if (data.time && data.time.length > 0) {
                        updateHUDs(data.time.length - 1);
                    }
                } else {
                    alert("Simulation error: " + data.message);
                }
            } catch (err) {
                console.error(err);
                alert("Failed to communicate with simulation server.");
            } finally {
                hideLoading();
            }
        }

        function updateKPIs(data) {
            // Calculate KPIs from arrays
            const maxG = Math.max(...data.g_force).toFixed(2);
            const maxTemp = Math.max(...data.motor_temp).toFixed(1);
            const finalSoc = parseFloat(data.soc[data.soc.length - 1]).toFixed(1);
            const maxError = Math.max(...data.tracking_error).toFixed(1);

            document.getElementById('val-gforce').innerText = maxG;
            document.getElementById('val-temp').innerText = maxTemp;
            document.getElementById('val-soc').innerText = finalSoc;
            document.getElementById('val-error').innerText = maxError;
        }

        function renderPlots(data) {
            // Colors matching our dashboard variables
            const cyan = '#06b6d4';
            const crimson = '#ef4444';
            const amber = '#f59e0b';
            const purple = '#8b5cf6';
            const green = '#10b981';
            const blue = '#60a5fa';
            const textGray = '#9ca3af';

            // --- PLOT 0: Static Propulsion Curves ---
            if (data.static_curve) {
                const traceStaticThrust = {
                    x: data.static_curve.current_motor,
                    y: data.static_curve.thrust_g,
                    name: 'Thrust (g)',
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: cyan, width: 3 },
                    yaxis: 'y'
                };

                const traceStaticEff = {
                    x: data.static_curve.current_motor,
                    y: data.static_curve.efficiency,
                    name: 'Efficiency (g/W)',
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: amber, width: 3, dash: 'dash' },
                    yaxis: 'y2'
                };

                const layoutStatic = {
                    paper_bgcolor: 'rgba(0,0,0,0)',
                    plot_bgcolor: 'rgba(0,0,0,0)',
                    font: { color: textGray, family: 'Outfit, sans-serif' },
                    margin: { t: 35, b: 45, l: 65, r: 65 },
                    showlegend: true,
                    legend: { x: 0.1, y: 1.15, orientation: 'h' },
                    xaxis: {
                        title: 'Motor Current (A)',
                        gridcolor: 'rgba(6,182,212,0.06)',
                        linecolor: 'rgba(6,182,212,0.1)'
                    },
                    yaxis: {
                        title: 'Thrust (g)',
                        gridcolor: 'rgba(6,182,212,0.06)',
                        linecolor: 'rgba(6,182,212,0.1)'
                    },
                    yaxis2: {
                        title: 'Efficiency (g/W)',
                        overlaying: 'y',
                        side: 'right',
                        showgrid: false
                    }
                };

                Plotly.newPlot('plot-static', [traceStaticThrust, traceStaticEff], layoutStatic, { responsive: true });
            }

            // --- PLOT 1: 3D Trajectory Overlay ---
            const traceRocket = {
                x: data.rocket_x,
                y: data.rocket_y,
                z: data.rocket_z,
                type: 'scatter3d',
                mode: 'lines',
                line: {
                    color: crimson,
                    width: 5,
                    dash: 'dash'
                },
                name: 'Rocket Path'
            };

            const traceDrone = {
                x: data.x,
                y: data.y,
                z: data.z,
                type: 'scatter3d',
                mode: 'lines',
                line: {
                    color: cyan,
                    width: 6
                },
                name: 'Drone Path'
            };

            let traces3d = [traceRocket, traceDrone];

            // Overlay 3D green crosses + line-of-sights at 4 timestamps
            const targetTimes = [0.696, 8.0, 16.643, 100.0];
            targetTimes.forEach((t) => {
                let idx = data.time.findIndex(timeVal => timeVal >= t);
                if (idx === -1) idx = data.time.length - 1;

                const dx = data.x[idx];
                const dy = data.y[idx];
                const dz = data.z[idx];

                const rx = data.rocket_x[idx];
                const ry = data.rocket_y[idx];
                const rz = data.rocket_z[idx];

                const crossSize = 15.0; // Size of cross arms in meters

                // X Arm
                traces3d.push({
                    x: [dx - crossSize, dx + crossSize],
                    y: [dy, dy],
                    z: [dz, dz],
                    type: 'scatter3d',
                    mode: 'lines',
                    line: { color: '#22c55e', width: 4 },
                    showlegend: false
                });

                // Y Arm
                traces3d.push({
                    x: [dx, dx],
                    y: [dy - crossSize, dy + crossSize],
                    z: [dz, dz],
                    type: 'scatter3d',
                    mode: 'lines',
                    line: { color: '#22c55e', width: 4 },
                    showlegend: false
                });

                // Z Arm
                traces3d.push({
                    x: [dx, dx],
                    y: [dy, dy],
                    z: [dz - crossSize, dz + crossSize],
                    type: 'scatter3d',
                    mode: 'lines',
                    line: { color: '#22c55e', width: 4 },
                    showlegend: false
                });

                // Camera Line of Sight (LOS) to Rocket
                traces3d.push({
                    x: [dx, rx],
                    y: [dy, ry],
                    z: [dz, rz],
                    type: 'scatter3d',
                    mode: 'lines',
                    line: { color: 'rgba(34, 197, 94, 0.45)', width: 2.5, dash: 'dot' },
                    showlegend: false
                });

                // Time label marker
                traces3d.push({
                    x: [dx],
                    y: [dy],
                    z: [dz],
                    type: 'scatter3d',
                    mode: 'markers+text',
                    marker: { size: 3, color: '#22c55e' },
                    text: [`t = ${t}s`],
                    textposition: 'top center',
                    textfont: { color: '#ffffff', size: 10, family: 'Outfit, sans-serif' },
                    showlegend: false
                });
            });

            const layout3d = {
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: textGray, family: 'Outfit, sans-serif' },
                margin: { t: 35, b: 45, l: 65, r: 65 },
                legend: { x: 0.1, y: 0.95, orientation: 'h' },
                scene: {
                    xaxis: { title: 'X (East/West, m)', gridcolor: 'rgba(6,182,212,0.08)', backgroundcolor: 'rgba(3,7,18,0.4)', showbackground: true },
                    yaxis: { title: 'Y (North/South, m)', gridcolor: 'rgba(6,182,212,0.08)', backgroundcolor: 'rgba(3,7,18,0.4)', showbackground: true },
                    zaxis: { title: 'Z (Altitude, m)', gridcolor: 'rgba(6,182,212,0.08)', backgroundcolor: 'rgba(3,7,18,0.4)', showbackground: true },
                    camera: {
                        eye: { x: 1.5, y: -1.5, z: 1.2 }
                    }
                }
            };

            Plotly.newPlot('plot-3d', traces3d, layout3d, { responsive: true });

            // --- PLOT 2: Stacked Multi-Physics Telemetry Charts ---
            
            // Chart A Traces (Kinematics)
            const traceVelocity = {
                x: data.time,
                y: data.velocity,
                name: 'Velocity (m/s)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#06b6d4', width: 3 }, // Neon Cyan
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceHSpan = {
                x: data.time,
                y: data.span_horizontal,
                name: 'Horiz Span (m)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#ef4444', width: 2, dash: 'dash' }, // Crimson
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceAccel = {
                x: data.time,
                y: data.g_force,
                name: 'Acceleration (G)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#f59e0b', width: 2.5 }, // Gold
                yaxis: 'y2',
                hoverinfo: 'none'
            };

            const traceLSpan = {
                x: data.time,
                y: data.span_longitudinal,
                name: 'Longit Span (m)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#c084fc', width: 2, dash: 'dot' }, // Purple
                yaxis: 'y2',
                hoverinfo: 'none'
            };

            // Chart B Traces (Power)
            const traceVoltage = {
                x: data.time,
                y: data.voltage_sag,
                name: 'Voltage Sag (V)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#22d3ee', width: 2.5 }, // Cyan
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceSOC = {
                x: data.time,
                y: data.soc,
                name: 'Battery SOC (%)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#10b981', width: 2, dash: 'dash' }, // Emerald
                yaxis: 'y2',
                hoverinfo: 'none'
            };

            // Chart C Traces (Attitude)
            const tracePitch = {
                x: data.time,
                y: data.pitch,
                name: 'Pitch (deg)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#f87171', width: 2 }, // Crimson-Orange
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceRoll = {
                x: data.time,
                y: data.roll,
                name: 'Roll (deg)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#60a5fa', width: 2, dash: 'dash' }, // Blue
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceYaw = {
                x: data.time,
                y: data.yaw,
                name: 'Yaw (deg)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#c084fc', width: 2, dash: 'dot' }, // Purple
                yaxis: 'y',
                hoverinfo: 'none'
            };

            const traceMotorTemp = {
                x: data.time,
                y: data.motor_temp,
                name: 'Motor Temp (°C)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#f59e0b', width: 2 }, // Amber/Gold
                yaxis: 'y2',
                hoverinfo: 'none'
            };

            // Layout definitions
            const layoutKinematics = {
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: textGray, family: 'Outfit, sans-serif' },
                margin: { t: 35, b: 45, l: 65, r: 65 },
                showlegend: true,
                legend: { x: 0.05, y: 1.15, orientation: 'h' },
                hovermode: 'x',
                xaxis: { 
                    domain: [0.08, 0.92],
                    gridcolor: 'rgba(6,182,212,0.06)', 
                    linecolor: 'rgba(6,182,212,0.1)', 
                    title: 'Mission Duration (seconds)' 
                },
                yaxis: { 
                    title: 'Velocity (m/s) & Horiz Span (m)', 
                    titlefont: { color: '#06b6d4' },
                    tickfont: { color: '#06b6d4' },
                    gridcolor: 'rgba(6, 182, 212, 0.08)',
                    linecolor: 'rgba(6, 182, 212, 0.1)',
                    side: 'left'
                },
                yaxis2: { 
                    title: 'Acceleration (G) & Longit Span (m)', 
                    titlefont: { color: '#f59e0b' },
                    tickfont: { color: '#f59e0b' },
                    overlaying: 'y', 
                    side: 'right', 
                    showgrid: false
                }
            };

            const layoutPower = {
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: textGray, family: 'Outfit, sans-serif' },
                margin: { t: 35, b: 45, l: 65, r: 65 },
                showlegend: true,
                legend: { x: 0.05, y: 1.15, orientation: 'h' },
                hovermode: 'x',
                xaxis: { 
                    domain: [0.08, 0.92],
                    gridcolor: 'rgba(6,182,212,0.06)', 
                    linecolor: 'rgba(6,182,212,0.1)', 
                    title: 'Mission Duration (seconds)' 
                },
                yaxis: { 
                    title: 'Voltage Sag (V)', 
                    titlefont: { color: '#22d3ee' },
                    tickfont: { color: '#22d3ee' },
                    gridcolor: 'rgba(6,182,212,0.06)',
                    linecolor: 'rgba(6,182,212,0.1)',
                    side: 'left'
                },
                yaxis2: { 
                    title: 'Battery SOC (%)', 
                    titlefont: { color: '#10b981' },
                    tickfont: { color: '#10b981' },
                    overlaying: 'y', 
                    side: 'right', 
                    showgrid: false,
                    range: [0, 105]
                }
            };

            const layoutAttitude = {
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: textGray, family: 'Outfit, sans-serif' },
                margin: { t: 35, b: 45, l: 65, r: 65 },
                showlegend: true,
                legend: { x: 0.05, y: 1.15, orientation: 'h' },
                hovermode: 'x',
                xaxis: { 
                    domain: [0.08, 0.92],
                    gridcolor: 'rgba(6,182,212,0.06)', 
                    linecolor: 'rgba(6,182,212,0.1)', 
                    title: 'Mission Duration (seconds)' 
                },
                yaxis: { 
                    title: 'Angles (deg)', 
                    titlefont: { color: textGray },
                    tickfont: { color: textGray },
                    gridcolor: 'rgba(6,182,212,0.06)',
                    linecolor: 'rgba(6,182,212,0.1)',
                    range: [-185, 185],
                    side: 'left'
                },
                yaxis2: {
                    title: 'Motor Temp (°C)',
                    titlefont: { color: '#f59e0b' },
                    tickfont: { color: '#f59e0b' },
                    overlaying: 'y',
                    side: 'right',
                    showgrid: false
                }
            };

            // Render the 3 separate plots
            Plotly.newPlot('plot-kinematics', [traceVelocity, traceHSpan, traceAccel, traceLSpan], layoutKinematics, { responsive: true });
            Plotly.newPlot('plot-power', [traceVoltage, traceSOC], layoutPower, { responsive: true });
            Plotly.newPlot('plot-attitude', [tracePitch, traceRoll, traceYaw, traceMotorTemp], layoutAttitude, { responsive: true });

            // Synchronized hover events
            const telemetryCharts = ['plot-kinematics', 'plot-power', 'plot-attitude'];
            telemetryCharts.forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.removeAllListeners && el.removeAllListeners('plotly_hover');
                    el.on('plotly_hover', function(eventData) {
                        if (!eventData || !eventData.points || eventData.points.length === 0) return;
                        const hoveredTime = eventData.points[0].x;
                        
                        // Find closest time index
                        let closestIdx = 0;
                        let minDiff = Infinity;
                        for (let i = 0; i < data.time.length; i++) {
                            let diff = Math.abs(data.time[i] - hoveredTime);
                            if (diff < minDiff) {
                                minDiff = diff;
                                closestIdx = i;
                            }
                        }
                        updateHUDs(closestIdx);
                    });
                }
            });

            // Linked X-axis zoom & pan synchronization
            let isRelayouting = false;
            telemetryCharts.forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    el.on('plotly_relayout', function(eventData) {
                        if (isRelayouting) return;
                        
                        let update = {};
                        let hasXupdate = false;
                        
                        if (eventData['xaxis.range[0]'] !== undefined && eventData['xaxis.range[1]'] !== undefined) {
                            update['xaxis.range[0]'] = eventData['xaxis.range[0]'];
                            update['xaxis.range[1]'] = eventData['xaxis.range[1]'];
                            hasXupdate = true;
                        } else if (eventData['xaxis.autorange'] !== undefined) {
                            update['xaxis.autorange'] = eventData['xaxis.autorange'];
                            hasXupdate = true;
                        }
                        
                        if (hasXupdate) {
                            isRelayouting = true;
                            const promises = telemetryCharts
                                .filter(otherId => otherId !== id)
                                .map(otherId => Plotly.relayout(otherId, update));
                            
                            Promise.all(promises).then(() => {
                                isRelayouting = false;
                            }).catch(() => {
                                isRelayouting = false;
                            });
                        }
                    });
                }
            });
        }

        // Helper to update HUD readout values across all 3 charts
        function updateHUDs(idx) {
            if (!currentSimData || idx < 0 || idx >= currentSimData.time.length) return;

            const timeVal = currentSimData.time[idx];
            const velocity = currentSimData.velocity[idx];
            const gForce = currentSimData.g_force[idx];
            const hSpan = currentSimData.span_horizontal[idx];
            const lSpan = currentSimData.span_longitudinal[idx];
            const voltage = currentSimData.voltage_sag[idx];
            const soc = currentSimData.soc[idx];
            const pitch = currentSimData.pitch[idx];
            const roll = currentSimData.roll[idx];
            const yaw = currentSimData.yaw[idx];

            // Chart A HUD updates
            document.getElementById('hud-a-time').innerText = timeVal.toFixed(1);
            document.getElementById('hud-a-vel').innerText = velocity.toFixed(1);
            document.getElementById('hud-a-gforce').innerText = gForce.toFixed(2);
            document.getElementById('hud-a-hspan').innerText = hSpan.toFixed(1);
            document.getElementById('hud-a-lspan').innerText = lSpan.toFixed(1);

            // Chart B HUD updates
            document.getElementById('hud-b-time').innerText = timeVal.toFixed(1);
            document.getElementById('hud-b-vol').innerText = voltage.toFixed(2);
            document.getElementById('hud-b-soc').innerText = soc.toFixed(1);

            // Chart C HUD updates
            document.getElementById('hud-c-time').innerText = timeVal.toFixed(1);
            document.getElementById('hud-c-pitch').innerText = pitch.toFixed(1);
            document.getElementById('hud-c-roll').innerText = roll.toFixed(1);
            document.getElementById('hud-c-yaw').innerText = yaw.toFixed(1);

            // Trigger neon orange flashing micro-indicators on threshold violations
            // 1. Velocity Warning (> 50 m/s)
            const cardVel = document.getElementById('hud-card-vel');
            const indVel = document.getElementById('ind-vel');
            if (velocity > 50) {
                cardVel.classList.add('alert');
                indVel.className = 'hud-indicator warning';
            } else {
                cardVel.classList.remove('alert');
                indVel.className = 'hud-indicator safe';
            }

            // 2. G-Force Warning (> 4.5 G)
            const cardG = document.getElementById('hud-card-gforce');
            const indG = document.getElementById('ind-gforce');
            if (gForce > 4.5) {
                cardG.classList.add('alert');
                indG.className = 'hud-indicator warning';
            } else {
                cardG.classList.remove('alert');
                indG.className = 'hud-indicator safe';
            }

            // 3. Voltage Warning (< 21.0 V)
            const cardVol = document.getElementById('hud-card-vol');
            const indVol = document.getElementById('ind-vol');
            if (voltage < 21.0) {
                cardVol.classList.add('alert');
                indVol.className = 'hud-indicator warning';
            } else {
                cardVol.classList.remove('alert');
                indVol.className = 'hud-indicator safe';
            }

            // 4. Battery SOC Warning (< 20%)
            const cardSoc = document.getElementById('hud-card-soc');
            const indSoc = document.getElementById('ind-soc');
            if (soc < 20) {
                cardSoc.classList.add('alert');
                indSoc.className = 'hud-indicator warning';
            } else {
                cardSoc.classList.remove('alert');
                indSoc.className = 'hud-indicator safe';
            }
        }

        // Auto trigger simulation run on load
        window.onload = function() {
            attachArchitectListeners();
            recalculateAircraftDesignSpecs();
            runSimulation();
        };
    </script>
</body>
</html>
"""


class SimulationRequestHandler(SimpleHTTPRequestHandler):
    """
    HTTP Request Handler serving the Ground Station UI
    and handling the dynamic trajectory simulation endpoint.
    """

    def log_message(self, format_str, *args):
        # Override to suppress standard HTTP request spam in terminal stdout
        pass

    def do_OPTIONS(self):
        """Handles CORS preflight OPTIONS requests."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        """Serves the main dashboard page inline at GET /."""
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
        else:
            # Fallback to default static file serving
            super().do_GET()

    def do_POST(self):
        """Handles API requests for simulation."""
        if self.path == '/api/simulate':
            self._handle_simulate()
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_simulate(self):
        """Processes simulation requests, runs 180s trajectory planner and returns results."""
        try:
            import numpy as np

            # Read inputs (optionally sent by frontend)
            content_length = int(self.headers.get('Content-Length', 0))
            payload = {}
            if content_length > 0:
                post_data = self.rfile.read(content_length)
                payload = json.loads(post_data.decode('utf-8'))

            # Extract parameters with defaults
            frame_w = float(payload.get('frame_weight', 184.0))
            arm_l = float(payload.get('arm_length', 280.0))
            lidar_w = float(payload.get('lidar_weight', 45.0))
            camera_w = float(payload.get('camera_weight', 215.0))
            battery_cap = float(payload.get('battery_capacity', 6500.0))
            battery_cells = int(payload.get('battery_cells', 6))
            battery_r = float(payload.get('battery_r', 0.003))
            gimbal_axes = int(payload.get('gimbal_axes', 3))
            
            motor_kv = float(payload.get('motor_kv', 1300.0))
            motor_r = float(payload.get('motor_r', 0.021))
            esc_current = float(payload.get('esc_current', 80.0))
            esc_r = float(payload.get('esc_r', 0.0015))
            prop_diameter = float(payload.get('prop_diameter', 7.0))
            prop_pitch = float(payload.get('prop_pitch', 5.0))
            prop_blades = int(payload.get('prop_blades', 3))

            # Configure virtual components based on the architect inputs
            battery = BatteryConfig(
                cells_s=battery_cells,
                capacity_mah=battery_cap,
                r_cell=battery_r,
                weight_g=battery_cap * battery_cells * 0.012246
            )
            esc = ESCConfig(
                max_current=esc_current,
                r_esc=esc_r
            )
            motor = MotorConfig(
                kv=motor_kv,
                r_motor=motor_r,
                weight_g=80.0
            )
            propeller = PropellerConfig(
                diameter_in=prop_diameter,
                pitch_in=prop_pitch,
                blade_count=prop_blades,
                ct=0.1115,
                cp=0.0565
            )

            prop_sys = PropulsionSystem(
                battery=battery,
                esc=esc,
                motor=motor,
                propeller=propeller,
                use_buck_model=True
            )

            # Mathematical takeoff mass calculation loop
            m_frame = frame_w
            m_arms = 4.0 * 20.0 * (arm_l / 280.0)
            m_motors = 4.0 * motor.weight_g
            m_battery = battery.weight_g
            m_lidar = lidar_w
            m_camera = camera_w
            m_gimbal = 120.0 * (gimbal_axes / 3.0)
            m_landing_gear = 25.0
            m_esc = 45.0
            m_props = 4.0 * 8.9
            m_pdb = 12.0
            m_bec = 8.0
            m_fc = 10.0
            m_imu = 2.0
            m_gps = 15.0
            m_vision = 25.0
            m_tof = 5.0
            m_rx = 3.0
            m_telemetry = 10.0
            m_vtx = 15.0

            total_component_mass_g = (
                m_frame + m_arms + m_motors + m_battery + m_lidar + m_vtx + 
                m_camera + m_gimbal + m_landing_gear + m_esc + m_props + 
                m_pdb + m_bec + m_fc + m_imu + m_gps + m_vision + m_tof + 
                m_rx + m_telemetry
            )
            # 15% allowance for wiring and fasteners
            auw_g = total_component_mass_g * 1.15

            # Load rocket profile and generate dynamic simulation natively
            rocket_data = load_rocket_profile("rocket_simulation.csv")
            
            drone_path = generate_optimal_drone_path(
                rocket_data, 
                propulsion_sys=prop_sys,
                override_mass_g=auw_g
            )

            # Generate benchmark static propulsion curves (100% SOC)
            static_throttles = np.linspace(0.0, 1.0, 21)
            static_thrust_g = []
            static_current_motor = []
            static_efficiency = []
            
            static_sys = PropulsionSystem(
                battery=battery,
                esc=esc,
                motor=motor,
                propeller=propeller,
                use_buck_model=True
            )
            for t_val in static_throttles:
                res_static = static_sys.evaluate(throttle=t_val, current_soc=1.0, execution_time=0.0)
                static_thrust_g.append(res_static["thrust_g"])
                static_current_motor.append(res_static["current_motor_a"])
                static_efficiency.append(res_static["efficiency_g_w"])
            
            # Return paired time-series data
            self._send_json({
                "status": "success",
                "rocket_x": (-rocket_data["position_east"]).tolist(),  # X = -Position_East
                "rocket_y": rocket_data["position_north"].tolist(),    # Y = Position_North
                "rocket_z": rocket_data["altitude"].tolist(),          # Z = Altitude
                "x": drone_path["pos_x"].tolist(),                     # Drone X
                "y": drone_path["pos_y"].tolist(),                     # Drone Y
                "z": drone_path["pos_z"].tolist(),                     # Drone Z
                "time": rocket_data["time"].tolist(),
                "soc": (drone_path["soc"] * 100).tolist(),             # Battery SOC percentage (0-100%)
                "voltage_sag": drone_path["v_bat"].tolist(),
                "motor_temp": drone_path["motor_temp_rise"].tolist(),
                "throttle": drone_path["throttle"].tolist(),
                "g_force": drone_path["g_force"].tolist(),
                "tracking_error": drone_path["tracking_error"].tolist(),
                "gimbal_angle": drone_path["gimbal_pitch"].tolist(),
                "velocity": drone_path["velocity"].tolist(),
                "span_horizontal": drone_path["span_horizontal"].tolist(),
                "span_longitudinal": drone_path["span_longitudinal"].tolist(),
                "pitch": drone_path["pitch"].tolist(),
                "roll": drone_path["roll"].tolist(),
                "yaw": drone_path["yaw"].tolist(),
                "static_curve": {
                    "throttle": static_throttles.tolist(),
                    "thrust_g": static_thrust_g,
                    "current_motor": static_current_motor,
                    "efficiency": static_efficiency
                }
            })

        except Exception as e:
            self._send_json({
                "status": "error",
                "message": f"Simulation failure: {str(e)}"
            }, status_code=500)

    def _send_json(self, data: dict, status_code: int = 200):
        """Serializes and transmits the API response payload."""
        self.send_response(status_code)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))


def main():
    port = 8000
    server_address = ('127.0.0.1', port)
    
    try:
        httpd = HTTPServer(server_address, SimulationRequestHandler)
        print("==================================================================")
        print("     UAV INTERCEPT GROUND STATION WEB SERVER RUNNING SUCCESSFULLY ")
        print("==================================================================")
        print(f"Server URL: http://127.0.0.1:{port}")
        print("Close this terminal window to stop the server.")
        print("------------------------------------------------------------------")
        
        # Open web browser automatically to the dashboard URL
        webbrowser.open(f"http://127.0.0.1:{port}")
        
        # Start server execution loop
        httpd.serve_forever()
    except Exception as e:
        print(f"Error starting simulation server: {str(e)}")
        sys.exit(1)


if __name__ == '__main__':
    main()
