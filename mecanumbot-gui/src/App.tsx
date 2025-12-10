import {
  BrowserRouter,
  NavLink,
  Route,
  Routes,
  Navigate,
} from "react-router-dom"
import "./App.css"

import Dashboard from "./pages/Dashboard"
import Control from "./pages/Control"
import Recording from "./pages/Recording"
import Camera from "./pages/Camera"

import FloatingLines from "./components/Background/FloatingLines"

const NAV = [
  { to: "/dashboard", label: "Dashboard", end: true },
  { to: "/control", label: "Control", end: false },
  { to: "/recording", label: "Recording", end: false },
  { to: "/camera", label: "Camera", end: false },
] as const

function App() {
  const mode = localStorage.getItem("mb_mode") ?? "ros" // "mock" | "ros"
  return (
    <BrowserRouter>
      <div className="app-root">
        {/* Global background */}
        <div className="app-bg">
          <FloatingLines
            enabledWaves={["top", "middle", "bottom"]}
            lineCount={[10, 15, 20]}
            lineDistance={[8, 6, 4]}
            bendRadius={5.0}
            bendStrength={-0.5}
            interactive={true}
            parallax={true}
          />
        </div>

        {/* Foreground app */}
        <div className="app-shell">
          <header className="app-header">
            <div className="brand">
              <span className="brand-dot" />
              <span className="brand-text">Mecanumbot GUI</span>
            </div>

            {/* TOP NAV TABS */}
            <nav className="top-nav">
              {NAV.map((item) => (
                <NavLink
                  key={item.to}
                  to={item.to}
                  end={item.end}
                  className={({ isActive }) =>
                    `top-link ${isActive ? "active" : ""}`
                  }
                >
                  {item.label}
                </NavLink>
              ))}
            </nav>

            <div className="header-right">
              <span className="status-pill">
                {mode === "mock" ? "Mock mode" : "ROS mode"}
              </span>
            </div>
          </header>

          <main className="app-main">
            <Routes>
            {/* Redirect root to dashboard */}
            <Route path="/" element={<Navigate to="/dashboard" replace />} />

            {/* Real pages */}
            <Route path="/dashboard" element={<Dashboard />} />
            <Route path="/control" element={<Control />} />
            <Route path="/recording" element={<Recording />} />
            <Route path="/camera" element={<Camera />} />

            {/* Fallback */}
            <Route path="*" element={<Navigate to="/dashboard" replace />} />
          </Routes>
          </main>
        </div>
      </div>
    </BrowserRouter>
  )
}

export default App
