from __future__ import annotations

import argparse
import os
import sys
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class QuietRangeRequestHandler(SimpleHTTPRequestHandler):
    """Static docs server with byte-range video support and quiet disconnects."""

    def log_error(self, format: str, *args: object) -> None:
        message = format % args
        if "ConnectionResetError" in message or "BrokenPipeError" in message:
            return
        super().log_error(format, *args)

    def copyfile(self, source, outputfile) -> None:  # type: ignore[no-untyped-def]
        try:
            super().copyfile(source, outputfile)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def end_headers(self) -> None:
        self.send_header("Accept-Ranges", "bytes")
        super().end_headers()

    def send_head(self):  # type: ignore[no-untyped-def]
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()

        range_header = self.headers.get("Range")
        if not range_header:
            return super().send_head()

        try:
            file_size = os.path.getsize(path)
            start, end = self._parse_range(range_header, file_size)
            file_obj = open(path, "rb")
            file_obj.seek(start)

            self.send_response(HTTPStatus.PARTIAL_CONTENT)
            self.send_header("Content-type", self.guess_type(path))
            self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
            self.send_header("Content-Length", str(end - start + 1))
            self.end_headers()
            self.range_end = end
            return file_obj
        except (OSError, ValueError):
            return super().send_head()

    def _parse_range(self, header: str, file_size: int) -> tuple[int, int]:
        unit, _, byte_range = header.partition("=")
        if unit.strip() != "bytes":
            raise ValueError("Only byte ranges are supported")

        start_text, _, end_text = byte_range.partition("-")
        if start_text:
            start = int(start_text)
            end = int(end_text) if end_text else file_size - 1
        else:
            suffix_length = int(end_text)
            start = max(file_size - suffix_length, 0)
            end = file_size - 1

        if start < 0 or end < start or start >= file_size:
            raise ValueError("Invalid range")

        return start, min(end, file_size - 1)

    def do_GET(self) -> None:
        file_obj = self.send_head()
        if file_obj:
            try:
                end = getattr(self, "range_end", None)
                if end is not None:
                    remaining = end - file_obj.tell() + 1
                    while remaining > 0:
                        chunk = file_obj.read(min(64 * 1024, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
                else:
                    self.copyfile(file_obj, self.wfile)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                if hasattr(self, "range_end"):
                    delattr(self, "range_end")
                file_obj.close()

    def do_HEAD(self) -> None:
        file_obj = self.send_head()
        if file_obj:
            if hasattr(self, "range_end"):
                delattr(self, "range_end")
            file_obj.close()


class QuietThreadingHTTPServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address) -> None:  # type: ignore[no-untyped-def]
        _, exc, _ = sys.exc_info()
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the P3GASUS docs site locally.")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--host", default="localhost")
    args = parser.parse_args()

    os.chdir(os.path.dirname(__file__))
    server = QuietThreadingHTTPServer((args.host, args.port), QuietRangeRequestHandler)
    print(f"Serving docs at http://{args.host}:{args.port}/")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
