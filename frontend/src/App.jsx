import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import Map2DView from './components/Map2DView.jsx'

import {
  AppBar,
  Alert,
  Box,
  Button,
  Chip,
  Container,
  Divider,
  Grid,
  IconButton,
  Paper,
  Toolbar,
  Stack,
  Tooltip,
  Typography,
} from '@mui/material'

import {
  ArrowBack,
  ArrowDownward,
  ArrowForward,
  ArrowUpward,
  Map as MapIcon,
  StopCircle,
  Wifi,
  WifiOff,
} from '@mui/icons-material'

// Potholes can add CPU/network load (map overlays + list rendering + polling).
// Enable by default; disable with: VITE_ENABLE_POTHOLES=false
const ENABLE_POTHOLES = (() => {
  const v = String(import.meta.env.VITE_ENABLE_POTHOLES ?? 'true').trim().toLowerCase()
  return !(v === '0' || v === 'false' || v === 'no' || v === 'off')
})()

const DEBUG_TELEOP = (() => {
  const v = String(import.meta.env.VITE_DEBUG_TELEOP ?? '').trim().toLowerCase()
  return v === '1' || v === 'true' || v === 'yes' || v === 'on'
})()

const TELEOP_AUTO_STOP_MS = (() => {
  // Default pulse duration (ms). Override with VITE_TELEOP_AUTO_STOP_MS.
  const raw = String(import.meta.env.VITE_TELEOP_AUTO_STOP_MS ?? '333').trim()
  const n = Number(raw)
  // 0 disables auto-stop.
  if (!Number.isFinite(n) || n < 0) return 1000
  return Math.floor(n)
})()

const POTHOLES_MAX_RENDER = 120

function App() {
  const [health, setHealth] = useState(null)
  const [healthError, setHealthError] = useState('')

  const [cmdVelStatus, setCmdVelStatus] = useState({
    enabled: false,
    is_connected: false,
    topic: '/cmd_vel',
    message_type: 'geometry_msgs/Twist',
    active: false,
    last_received_at: null,
    last_published_at: null,
  })

  const [cmdVelError, setCmdVelError] = useState('')

  const [potholes, setPotholes] = useState([])
  const [potholesError, setPotholesError] = useState('')
  const potholesSinceSeqRef = useRef(null)

  const [_command, setCommand] = useState({ linear_x: 0, angular_z: 0 })
  const inFlightRef = useRef(false)
  const [teleopActiveKey, setTeleopActiveKey] = useState(null)
  const teleopActiveKeyRef = useRef(null)
  const teleopAutoStopTimerRef = useRef(null)

  const TELEOP = useMemo(
    () => ({
      // Continuous publishing, including zeros.
      publishHz: 15,
      linear: 0.5,
      angular: 1.0,
    }),
    [],
  )

  const callCmdVel = async (path, body) => {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body ? JSON.stringify(body) : '{}',
    })

    if (!res.ok) {
      let detail = ''
      try {
        const data = await res.json()
        detail = data?.error ? String(data.error) : JSON.stringify(data)
      } catch {
        try {
          detail = await res.text()
        } catch {
          detail = ''
        }
      }
      throw new Error(`cmd_vel HTTP ${res.status}${detail ? `: ${detail}` : ''}`)
    }

    // Only log cmd_vel commands that were actually sent successfully.
    console.log('[cmd_vel sent]', path, body || {})
  }

  const holdCmdVel = async (linear_x, angular_z) => {
    await callCmdVel('/api/ros/cmd_vel/hold', { linear_x, angular_z })
  }

  const releaseCmdVel = useCallback(async () => {
    await callCmdVel('/api/ros/cmd_vel/release')
  }, [])

  const clearTeleopAutoStop = () => {
    if (teleopAutoStopTimerRef.current) {
      clearTimeout(teleopAutoStopTimerRef.current)
      teleopAutoStopTimerRef.current = null
    }
  }

  const stopTeleop = () => {
    clearTeleopAutoStop()
    teleopActiveKeyRef.current = null
    setTeleopActiveKey(null)
    setCommandAndLog(0, 0)
    releaseCmdVel().catch((err) => setCmdVelError(String(err)))
  }

  const toggleCmdVel = async (key, linear_x, angular_z) => {
    // Toggle off if the same button is pressed again.
    const shouldStop = teleopActiveKey === key

    // Avoid overlapping HTTP calls when clicking rapidly.
    if (inFlightRef.current) return
    inFlightRef.current = true

    // Any new user action cancels the previous auto-stop countdown.
    clearTeleopAutoStop()

    try {
      if (shouldStop) {
        teleopActiveKeyRef.current = null
        setTeleopActiveKey(null)
        setCommandAndLog(0, 0)
        await releaseCmdVel()
      } else {
        teleopActiveKeyRef.current = key
        setTeleopActiveKey(key)
        setCommandAndLog(linear_x, angular_z)
        await holdCmdVel(linear_x, angular_z)

        // Auto-stop after a fixed duration (default 1s) to return to zeros.
        if (TELEOP_AUTO_STOP_MS > 0) {
          teleopAutoStopTimerRef.current = setTimeout(() => {
            if (teleopActiveKeyRef.current !== key) return
            stopTeleop()
          }, TELEOP_AUTO_STOP_MS)
        }
      }
      setCmdVelError('')
    } catch (err) {
      setCmdVelError(String(err))
    } finally {
      inFlightRef.current = false
    }
  }

  const setCommandAndLog = (linear_x, angular_z) => {
    const next = { linear_x, angular_z }
    if (DEBUG_TELEOP) console.log('[teleop] set command', next)
    setCommand(next)
  }

  useEffect(() => {
    const run = async () => {
      try {
        setHealthError('')
        const res = await fetch('/api/health')
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`)
        }
        const data = await res.json()
        setHealth(data)
      } catch (err) {
        setHealth(null)
        setHealthError(String(err))
      }
    }

    run()
  }, [])

  useEffect(() => {
    if (!ENABLE_POTHOLES) return

    let cancelled = false

    const poll = async () => {
      try {
        setPotholesError('')
        const since = potholesSinceSeqRef.current
        const url = since ? `/api/potholes?limit=500&since_seq=${encodeURIComponent(String(since))}` : '/api/potholes?limit=500'
        const res = await fetch(url)
        if (!res.ok) throw new Error(`potholes HTTP ${res.status}`)
        const data = await res.json()
        const items = Array.isArray(data?.items) ? data.items : []
        const nextSince = data?.next_since_seq ?? null
        if (!cancelled) {
          setPotholes((prev) => {
            if (!since) return items
            if (!items.length) return prev
            const merged = [...prev, ...items]
            // Keep bounded client-side too.
            return merged.length > 500 ? merged.slice(merged.length - 500) : merged
          })
          if (nextSince != null) potholesSinceSeqRef.current = nextSince
        }
      } catch (e) {
        if (!cancelled) setPotholesError(String(e))
      }
    }

    poll()
    const id = setInterval(poll, 1000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [])

  useEffect(() => {
    let cancelled = false
    const poll = async () => {
      try {
        const res = await fetch('/api/ros/cmd_vel/status')
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`)
        }
        const data = await res.json()
        if (DEBUG_TELEOP) console.log('[teleop] status', data)
        if (!cancelled) setCmdVelStatus(data)
      } catch {
        if (!cancelled) {
          setCmdVelStatus((prev) => ({
            ...prev,
            enabled: false,
            active: false,
            is_connected: false,
          }))
        }
      }
    }

    poll()
    const id = setInterval(poll, 250)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [])

  useEffect(() => {
    // Ensure we release on unmount/navigation.
    return () => {
      releaseCmdVel().catch(() => {})
    }
  }, [releaseCmdVel])

  const indicatorText = useMemo(() => {
    if (!cmdVelStatus?.enabled) return 'ROS disabled (ENABLE_ROSBRIDGE=false)'
    if (!cmdVelStatus?.is_connected) return 'ROS not connected'
    if (cmdVelStatus?.active) return `ROS OK: ${cmdVelStatus.topic} active`
    return `No ${cmdVelStatus.topic}`
  }, [cmdVelStatus])

  const rosChip = useMemo(() => {
    if (!cmdVelStatus?.enabled) {
      return { label: 'ROS disabled', color: 'default', icon: <WifiOff fontSize="small" /> }
    }
    if (!cmdVelStatus?.is_connected) {
      return { label: 'ROS disconnected', color: 'warning', icon: <WifiOff fontSize="small" /> }
    }
    return { label: 'ROS connected', color: 'success', icon: <Wifi fontSize="small" /> }
  }, [cmdVelStatus])

  const backendChip = useMemo(() => {
    if (healthError) return { label: 'Backend error', color: 'error' }
    if (!health) return { label: 'Backend loading', color: 'default' }
    return { label: 'Backend OK', color: 'success' }
  }, [health, healthError])

  const potholesForList = useMemo(() => {
    if (!ENABLE_POTHOLES) return []
    if (!Array.isArray(potholes) || !potholes.length) return []
    const start = Math.max(0, potholes.length - POTHOLES_MAX_RENDER)
    // Show newest first without copying/reversing the entire 500-element array each render.
    const slice = potholes.slice(start)
    slice.reverse()
    return slice
  }, [potholes])

  return (
    <Box sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <AppBar position="sticky" color="transparent" elevation={0}>
        <Toolbar sx={{ minHeight: 68 }}>
          <Stack direction="row" spacing={1.25} alignItems="center" sx={{ flex: 1, minWidth: 0 }}>
            <MapIcon fontSize="small" />
            <Box sx={{ minWidth: 0 }}>
              <Typography variant="h6" sx={{ lineHeight: 1.1 }} noWrap>
                RoadWatch Console
              </Typography>
              <Typography variant="caption" color="text.secondary" noWrap>
                Map • Teleop • Pothole detections
              </Typography>
            </Box>
          </Stack>

          <Stack direction="row" spacing={1} alignItems="center">
            <Chip size="small" label={backendChip.label} color={backendChip.color} />
            <Chip size="small" icon={rosChip.icon} label={rosChip.label} color={rosChip.color} />
            {ENABLE_POTHOLES ? (
              <Chip size="small" label="Potholes enabled" color="secondary" variant="outlined" />
            ) : (
              <Chip size="small" label="Potholes disabled" color="default" variant="outlined" />
            )}
          </Stack>
        </Toolbar>
      </AppBar>

      <Container maxWidth="xl" sx={{ py: 2.5, flex: 1 }}>
        <Grid container spacing={2} alignItems="stretch">
          <Grid item xs={12} lg={8}>
            <Paper sx={{ p: 2, height: { xs: 'auto', lg: 'calc(100vh - 140px)' } }}>
              <Stack spacing={1.5} sx={{ height: '100%' }}>
                <Stack direction="row" alignItems="baseline" justifyContent="space-between" spacing={2}>
                  <Box>
                    <Typography variant="h6">2D Map</Typography>
                    <Typography variant="body2" color="text.secondary">
                      Live occupancy grid with robot pose + pothole markers
                    </Typography>
                  </Box>
                </Stack>

                <Box sx={{ flex: 1, minHeight: 360, height: { xs: 420, md: 520, lg: '100%' } }}>
                  <Map2DView potholes={ENABLE_POTHOLES ? potholes : []} />
                </Box>
              </Stack>
            </Paper>
          </Grid>

          <Grid item xs={12} lg={4}>
            <Stack spacing={2}>
              <Paper sx={{ p: 2 }}>
                <Stack spacing={1.25}>
                  <Stack direction="row" justifyContent="space-between" alignItems="center" spacing={2}>
                    <Box>
                      <Typography variant="h6">Teleop</Typography>
                      <Typography variant="body2" color="text.secondary">
                        {indicatorText}
                      </Typography>
                    </Box>
                    <Tooltip title="Stop (release /cmd_vel)">
                      <span>
                        <Button
                          color="error"
                          variant="contained"
                          startIcon={<StopCircle />}
                          onClick={() => stopTeleop()}
                        >
                          Stop
                        </Button>
                      </span>
                    </Tooltip>
                  </Stack>

                  {cmdVelError ? <Alert severity="error">{cmdVelError}</Alert> : null}

                  <Divider />

                  <Box
                    sx={{
                      display: 'grid',
                      gridTemplateColumns: '56px 56px 56px',
                      gap: 1,
                      justifyContent: 'start',
                      alignItems: 'center',
                    }}
                  >
                    <Box />
                    <Tooltip title="Forward">
                      <IconButton
                        onClick={() => toggleCmdVel('forward', TELEOP.linear, 0)}
                        color={teleopActiveKey === 'forward' ? 'primary' : 'default'}
                        sx={{ border: '1px solid', borderColor: 'divider' }}
                      >
                        <ArrowUpward />
                      </IconButton>
                    </Tooltip>
                    <Box />

                    <Tooltip title="Turn left">
                      <IconButton
                        onClick={() => toggleCmdVel('left', 0, TELEOP.angular)}
                        color={teleopActiveKey === 'left' ? 'primary' : 'default'}
                        sx={{ border: '1px solid', borderColor: 'divider' }}
                      >
                        <ArrowBack />
                      </IconButton>
                    </Tooltip>

                    <Tooltip title="Release">
                      <IconButton
                        onClick={() => stopTeleop()}
                        color="error"
                        sx={{ border: '1px solid', borderColor: 'divider' }}
                      >
                        <StopCircle />
                      </IconButton>
                    </Tooltip>

                    <Tooltip title="Turn right">
                      <IconButton
                        onClick={() => toggleCmdVel('right', 0, -TELEOP.angular)}
                        color={teleopActiveKey === 'right' ? 'primary' : 'default'}
                        sx={{ border: '1px solid', borderColor: 'divider' }}
                      >
                        <ArrowForward />
                      </IconButton>
                    </Tooltip>

                    <Box />
                    <Tooltip title="Backward">
                      <IconButton
                        onClick={() => toggleCmdVel('backward', -TELEOP.linear, 0)}
                        color={teleopActiveKey === 'backward' ? 'primary' : 'default'}
                        sx={{ border: '1px solid', borderColor: 'divider' }}
                      >
                        <ArrowDownward />
                      </IconButton>
                    </Tooltip>
                    <Box />
                  </Box>

                  <Divider />

                  <Box>
                    <Typography variant="caption" color="text.secondary" display="block">
                      last_rx: <Box component="span" sx={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace' }}>{cmdVelStatus?.last_received_at || '—'}</Box>
                    </Typography>
                    <Typography variant="caption" color="text.secondary" display="block">
                      last_tx: <Box component="span" sx={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace' }}>{cmdVelStatus?.last_published_at || '—'}</Box>
                    </Typography>
                    <Typography variant="caption" color="text.secondary" display="block" sx={{ mt: 0.5 }}>
                      payload:{' '}
                      <Box component="span" sx={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace' }}>
                        {cmdVelStatus?.last_published_message ? JSON.stringify(cmdVelStatus.last_published_message) : '—'}
                      </Box>
                    </Typography>
                  </Box>
                </Stack>
              </Paper>

              {ENABLE_POTHOLES ? (
                <Paper sx={{ p: 2 }}>
                  <Stack spacing={1.25}>
                    <Stack direction="row" justifyContent="space-between" alignItems="baseline" spacing={2}>
                      <Box>
                        <Typography variant="h6">Potholes</Typography>
                        <Typography variant="body2" color="text.secondary">
                          Latest {potholes.length} detections (showing up to {POTHOLES_MAX_RENDER})
                        </Typography>
                      </Box>
                      <Chip size="small" label={potholes.length ? `${potholes.length}` : '0'} color={potholes.length ? 'secondary' : 'default'} />
                    </Stack>

                    {potholesError ? <Alert severity="warning">{potholesError}</Alert> : null}

                    <Box
                      sx={{
                        maxHeight: 340,
                        overflow: 'auto',
                        borderRadius: 2,
                        border: '1px solid',
                        borderColor: 'divider',
                        p: 1.25,
                      }}
                    >
                      {potholesForList.length ? (
                        <Stack spacing={1}>
                          {potholesForList.map((p) => (
                            <Box
                              key={p?.seq ?? `${p?.id ?? 'id'}-${p?.timestamp ?? 't'}`}
                              sx={{
                                p: 1,
                                borderRadius: 2,
                                border: '1px solid',
                                borderColor: 'divider',
                                background: 'rgba(255,255,255,0.03)',
                              }}
                            >
                              <Typography
                                variant="body2"
                                sx={{
                                  fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace',
                                  fontSize: 12,
                                }}
                              >
                                seq {p?.seq ?? '—'} · id {p?.id ?? '—'} · x {p?.x_coordinate ?? '—'} · t{' '}
                                {typeof p?.timestamp === 'number' ? p.timestamp.toFixed(2) : p?.timestamp ?? '—'}
                              </Typography>
                              <Typography variant="caption" color="text.secondary" display="block">
                                received_at: {p?.received_at ?? '—'} · source: {p?.source ?? '—'}
                              </Typography>
                            </Box>
                          ))}
                        </Stack>
                      ) : (
                        <Typography variant="body2" color="text.secondary" sx={{ p: 1 }}>
                          No potholes received yet. Start the detector with a server URL.
                        </Typography>
                      )}
                    </Box>

                    <Typography variant="caption" color="text.secondary">
                      Example:{' '}
                      <Box component="span" sx={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace' }}>
                        python3 camera_video.py --headless --server-url http://&lt;server_ip&gt;:5000 --send-exited
                      </Box>
                    </Typography>
                  </Stack>
                </Paper>
              ) : null}

              <Paper sx={{ p: 2 }}>
                <Stack spacing={1.25}>
                  <Typography variant="h6">Backend</Typography>
                  {healthError ? <Alert severity="error">{healthError}</Alert> : null}
                  <Box
                    sx={{
                      borderRadius: 2,
                      border: '1px solid',
                      borderColor: 'divider',
                      p: 1.25,
                      background: 'rgba(255,255,255,0.03)',
                    }}
                  >
                    <Typography
                      variant="body2"
                      sx={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace', fontSize: 12 }}
                    >
                      {health ? JSON.stringify(health, null, 2) : 'Loading…'}
                    </Typography>
                  </Box>
                </Stack>
              </Paper>
            </Stack>
          </Grid>
        </Grid>
      </Container>
    </Box>
  )
}

export default App
