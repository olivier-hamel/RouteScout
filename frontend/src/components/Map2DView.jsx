import { useEffect, useMemo, useRef, useState } from 'react'

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v))

const yawFromQuat = (q) => {
  if (!q) return 0
  const x = Number(q.x ?? 0)
  const y = Number(q.y ?? 0)
  const z = Number(q.z ?? 0)
  const w = Number(q.w ?? 1)
  // yaw (Z) from quaternion
  const siny_cosp = 2 * (w * z + x * y)
  const cosy_cosp = 1 - 2 * (y * y + z * z)
  return Math.atan2(siny_cosp, cosy_cosp)
}

const lerp = (a, b, t) => a + (b - a) * t
const lerpRgb = (c0, c1, t) => [
  Math.round(lerp(c0[0], c1[0], t)),
  Math.round(lerp(c0[1], c1[1], t)),
  Math.round(lerp(c0[2], c1[2], t)),
]

function buildOccupancyImage(mapMsg) {
  const info = mapMsg?.info
  const data = mapMsg?.data
  if (!info || !Array.isArray(data)) return null

  const width = Number(info.width ?? 0)
  const height = Number(info.height ?? 0)
  const resolution = Number(info.resolution ?? 0)
  const origin = info.origin ?? { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }

  if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) return null
  if (!Number.isFinite(resolution) || resolution <= 0) return null

  // Dark-theme palette (unknown / free / occupied)
  const unknownRgb = [30, 41, 59] // slate-800
  const freeRgb = [11, 18, 32] // app background
  const occupiedRgb = [226, 232, 240] // slate-200

  // Precompute image in map pixel coordinates (x right, y up) but stored in canvas coords (y down).
  const off = document.createElement('canvas')
  off.width = width
  off.height = height
  const ctx = off.getContext('2d', { willReadFrequently: true })
  const img = ctx.createImageData(width, height)

  // data is row-major starting at (0,0) at map origin (lower-left). We flip Y for canvas.
  for (let i = 0; i < data.length; i++) {
    const x = i % width
    const y = Math.floor(i / width)
    const yCanvas = height - 1 - y
    const idx = (yCanvas * width + x) * 4

    const occ = Number(data[i])

    let rgb = unknownRgb
    if (Number.isFinite(occ) && occ >= 0) {
      const v = clamp(occ, 0, 100)
      // 0 (free) -> freeRgb, 100 (occupied) -> occupiedRgb
      rgb = lerpRgb(freeRgb, occupiedRgb, v / 100)
    }

    img.data[idx + 0] = rgb[0]
    img.data[idx + 1] = rgb[1]
    img.data[idx + 2] = rgb[2]
    img.data[idx + 3] = 255
  }

  ctx.putImageData(img, 0, 0)

  return {
    canvas: off,
    width,
    height,
    resolution,
    origin,
  }
}

function worldToMapPixel({ x, y }, map) {
  const originPos = map.origin?.position ?? { x: 0, y: 0 }
  const originOri = map.origin?.orientation ?? { x: 0, y: 0, z: 0, w: 1 }
  const ox = Number(originPos.x ?? 0)
  const oy = Number(originPos.y ?? 0)
  const oyaw = yawFromQuat(originOri)

  const dx = x - ox
  const dy = y - oy

  // Rotate into map grid axes (inverse of origin rotation)
  const c = Math.cos(-oyaw)
  const s = Math.sin(-oyaw)
  const gx = (c * dx - s * dy) / map.resolution
  const gy = (s * dx + c * dy) / map.resolution

  const px = gx
  const py = map.height - 1 - gy
  return { px, py }
}

export default function Map2DView({ potholes = [] }) {
  const containerRef = useRef(null)
  const canvasRef = useRef(null)
  const mapSinceRef = useRef(null)

  const [viewport, setViewport] = useState({ width: 700, height: 520 })

  const [mapPacket, setMapPacket] = useState({ received_at: null, message: null })
  const [pose2d, setPose2d] = useState(null)
  const [error, setError] = useState('')

  useEffect(() => {
    const el = containerRef.current
    if (!el) return

    const ro = new ResizeObserver((entries) => {
      const cr = entries?.[0]?.contentRect
      if (!cr) return
      const width = Math.max(240, Math.floor(cr.width))
      const height = Math.max(240, Math.floor(cr.height))
      setViewport((prev) => {
        if (prev.width === width && prev.height === height) return prev
        return { width, height }
      })
    })

    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  const mapRender = useMemo(() => {
    if (!mapPacket?.message) return null
    return buildOccupancyImage(mapPacket.message)
  }, [mapPacket.message])

  useEffect(() => {
    let cancelled = false

    const pollMap = async () => {
      const since = mapSinceRef.current
      const url = since
        ? `/api/ros/map?since=${encodeURIComponent(String(since))}`
        : '/api/ros/map'
      const res = await fetch(url)
      if (res.status === 204) return
      if (!res.ok) throw new Error(`map HTTP ${res.status}`)
      const data = await res.json()
      if (cancelled) return

      const nextReceivedAt = data?.received_at ?? null
      const nextMessage = data?.message ?? null
      mapSinceRef.current = nextReceivedAt
      setMapPacket({ received_at: nextReceivedAt, message: nextMessage })
    }

    const pollPose = async () => {
      const res = await fetch('/api/ros/robot_pose_2d')
      if (!res.ok) throw new Error(`pose HTTP ${res.status}`)
      const data = await res.json()
      if (!cancelled) setPose2d(data)
    }

    // Map updates are large and usually infrequent; poll slowly.
    // Pose is small and can be polled faster.
    const tickMap = async () => {
      try {
        setError('')
        await pollMap()
      } catch (e) {
        if (!cancelled) setError(String(e))
      }
    }

    const tickPose = async () => {
      try {
        await pollPose()
      } catch (e) {
        if (!cancelled) setError(String(e))
      }
    }

    tickMap()
    tickPose()
    const mapId = setInterval(tickMap, 5000)
    const poseId = setInterval(tickPose, 300)
    return () => {
      cancelled = true
      clearInterval(mapId)
      clearInterval(poseId)
    }
  }, [])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const dpr = Math.max(1, Number(window.devicePixelRatio || 1))
    canvas.width = Math.max(1, Math.floor(viewport.width * dpr))
    canvas.height = Math.max(1, Math.floor(viewport.height * dpr))
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

    const W = viewport.width
    const H = viewport.height

    ctx.clearRect(0, 0, W, H)

    if (!mapRender) {
      ctx.fillStyle = 'rgba(255,255,255,0.06)'
      ctx.fillRect(0, 0, W, H)
      ctx.fillStyle = 'rgba(255,255,255,0.7)'
      ctx.font = '14px system-ui, sans-serif'
      ctx.fillText('Waiting for /map ...', 12, 24)
      return
    }

    // Fit mode: "cover" (fills the viewport, cropping if needed).
    const scale = Math.max(W / mapRender.width, H / mapRender.height)
    const drawW = mapRender.width * scale
    const drawH = mapRender.height * scale

    // Default center.
    let offX = (W - drawW) / 2
    let offY = (H - drawH) / 2

    // Follow the robot (keeps the interesting part of the map centered).
    if (pose2d?.ok) {
      const rx = Number(pose2d?.x ?? NaN)
      const ry = Number(pose2d?.y ?? NaN)
      if (Number.isFinite(rx) && Number.isFinite(ry)) {
        const { px, py } = worldToMapPixel({ x: rx, y: ry }, mapRender)
        const desiredX = W / 2 - px * scale
        const desiredY = H / 2 - py * scale

        // Clamp so we don't show blank space outside the map.
        const minX = W - drawW
        const minY = H - drawH
        offX = clamp(desiredX, minX, 0)
        offY = clamp(desiredY, minY, 0)
      }
    }

    ctx.imageSmoothingEnabled = false
    ctx.drawImage(mapRender.canvas, offX, offY, drawW, drawH)

    // Draw pothole markers (persistent).
    // Prefer server-provided robot pose snapshot; fall back to current pose if not present.
    if (Array.isArray(potholes) && potholes.length) {
      ctx.save()
      ctx.fillStyle = '#ff3b3b'
      for (const p of potholes) {
        const rp = p?.robot_pose
        const wx = Number((rp?.x ?? null) ?? NaN)
        const wy = Number((rp?.y ?? null) ?? NaN)
        const useFallback = !Number.isFinite(wx) || !Number.isFinite(wy)

        const x = useFallback ? Number(pose2d?.x ?? NaN) : wx
        const y = useFallback ? Number(pose2d?.y ?? NaN) : wy
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue

        const { px, py } = worldToMapPixel({ x, y }, mapRender)
        const sx = offX + px * scale
        const sy = offY + py * scale

        ctx.beginPath()
        ctx.arc(sx, sy, 6, 0, Math.PI * 2)
        ctx.fill()
      }
      ctx.restore()
    }

    // Draw robot overlay if we have pose.
    if (pose2d?.ok) {
      const robot = { x: Number(pose2d.x ?? 0), y: Number(pose2d.y ?? 0) }
      const yaw = Number(pose2d.yaw ?? 0)

      // If TF is unavailable, pose is in odom frame; we still draw it but label the canvas.
      const { px, py } = worldToMapPixel(robot, mapRender)
      const sx = offX + px * scale
      const sy = offY + py * scale

      // Robot arrow length in pixels
      const len = 18
      const dx = Math.cos(yaw)
      const dy = Math.sin(yaw)
      const ex = sx + dx * len
      const ey = sy - dy * len // invert Y for canvas

      ctx.save()
      ctx.lineWidth = 2
      ctx.strokeStyle = '#ff3b3b'
      ctx.fillStyle = '#ff3b3b'

      // Body
      ctx.beginPath()
      ctx.arc(sx, sy, 4, 0, Math.PI * 2)
      ctx.fill()

      // Heading
      ctx.beginPath()
      ctx.moveTo(sx, sy)
      ctx.lineTo(ex, ey)
      ctx.stroke()

      // Arrow head
      const head = 6
      const ang = Math.atan2(ey - sy, ex - sx)
      ctx.beginPath()
      ctx.moveTo(ex, ey)
      ctx.lineTo(ex - head * Math.cos(ang - Math.PI / 6), ey - head * Math.sin(ang - Math.PI / 6))
      ctx.lineTo(ex - head * Math.cos(ang + Math.PI / 6), ey - head * Math.sin(ang + Math.PI / 6))
      ctx.closePath()
      ctx.fill()

      ctx.restore()

      if (pose2d?.tf_available === false) {
        ctx.fillStyle = 'rgba(255,200,0,0.95)'
        ctx.font = '12px system-ui, sans-serif'
        ctx.fillText('TF unavailable: showing /odom pose (may not align with map)', 12, H - 12)
      }
    }
  }, [mapRender, pose2d, potholes])

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', minHeight: 0 }}>
      {error ? (
        <div
          style={{
            marginBottom: 10,
            padding: '10px 12px',
            borderRadius: 12,
            border: '1px solid rgba(255, 59, 59, 0.35)',
            background: 'rgba(255, 59, 59, 0.08)',
            color: 'rgba(255,255,255,0.92)',
            fontSize: 13,
          }}
        >
          Map error: {error}
        </div>
      ) : null}

      <div
        ref={containerRef}
        style={{
          width: '100%',
          flex: 1,
          minHeight: 0,
          borderRadius: 14,
          border: '1px solid rgba(255,255,255,0.10)',
          overflow: 'hidden',
          background: 'rgba(11,18,32,0.45)',
        }}
      >
        <canvas ref={canvasRef} style={{ width: '100%', height: '100%', display: 'block' }} />
      </div>

      <div style={{ marginTop: 8, fontSize: 12, opacity: 0.75 }}>
        /map: {mapPacket?.received_at || '—'} · pose: {pose2d?.frame || '—'} {pose2d?.tf_available === false ? '(no TF)' : ''}
      </div>
    </div>
  )
}
