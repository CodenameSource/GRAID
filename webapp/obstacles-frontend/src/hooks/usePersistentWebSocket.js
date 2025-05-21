import { useEffect, useRef } from "react";

/**
 * Opens a single WebSocket to `url`, and will
 * automatically reconnect with exponential backoff
 * if it ever closes or errors.
 *
 * @param {string} url
 * @param {Object}   handlers
 * @param {() => void}      [handlers.onOpen]
 * @param {(data:any)=>void} [handlers.onMessage]
 * @param {(err:Event)=>void}[handlers.onError]
 */
export function usePersistentWebSocket(
  url,
  { onOpen, onMessage, onError }
) {
  const wsRef = useRef(null);
  const attemptsRef = useRef(0);
  const timerRef = useRef(null);
  const shouldReconnect = useRef(true);

  const connect = () => {
    // Tear down any old socket before making a new one
    if (wsRef.current) {
      wsRef.current.onopen = wsRef.current.onmessage = wsRef.current.onerror = wsRef.current.onclose = null;
      wsRef.current.close();
    }

    const ws = new WebSocket(url);
    wsRef.current = ws;

    ws.onopen = () => {
      attemptsRef.current = 0;
      onOpen?.();
    };

    ws.onmessage = e => {
      try {
        onMessage?.(JSON.parse(e.data));
      } catch (err) {
        console.error("Invalid JSON from WS:", err);
      }
    };

    ws.onerror = e => {
      onError?.(e);
    };

    ws.onclose = ev => {
      if (!shouldReconnect.current) return;
      // exponential backoff: 1s, 2s, 4s, 8s… capped at 30s
      const delay = Math.min(30000, 1000 * 2 ** attemptsRef.current);
      attemptsRef.current += 1;
      timerRef.current = setTimeout(connect, delay);
    };
  };

  useEffect(() => {
    connect();

    return () => {
      // stop any future reconnects
      shouldReconnect.current = true;
      clearTimeout(timerRef.current);
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, [url]);

  // expose the socket if you ever need to send manually:
  return wsRef.current;
}
