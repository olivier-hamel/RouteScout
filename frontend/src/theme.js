import { createTheme } from '@mui/material/styles'

export function makeTheme(mode = 'dark') {
  return createTheme({
    palette: {
      mode,
      primary: { main: '#4F7CFF' },
      secondary: { main: '#22C55E' },
      background: {
        default: mode === 'dark' ? '#0B1220' : '#F6F7FB',
        paper: mode === 'dark' ? 'rgba(255,255,255,0.06)' : '#FFFFFF',
      },
    },
    shape: { borderRadius: 14 },
    typography: {
      fontFamily: [
        'Inter',
        'system-ui',
        '-apple-system',
        'Segoe UI',
        'Roboto',
        'Helvetica',
        'Arial',
        'Apple Color Emoji',
        'Segoe UI Emoji',
      ].join(','),
      h6: { fontWeight: 700 },
    },
    components: {
      MuiAppBar: {
        styleOverrides: {
          root: {
            backdropFilter: 'blur(12px)',
            borderBottom: '1px solid rgba(255,255,255,0.10)',
          },
        },
      },
      MuiPaper: {
        styleOverrides: {
          root: {
            border: '1px solid rgba(255,255,255,0.10)',
            backgroundImage: 'none',
          },
        },
      },
      MuiButton: {
        defaultProps: {
          disableElevation: true,
        },
      },
      MuiChip: {
        styleOverrides: {
          root: {
            fontWeight: 600,
          },
        },
      },
    },
  })
}
