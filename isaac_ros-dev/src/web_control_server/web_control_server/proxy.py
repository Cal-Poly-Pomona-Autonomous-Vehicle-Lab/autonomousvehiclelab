import requests
from flask import Flask, Response, request

app = Flask(__name__)

WEB_VIDEO_SERVER_BASE_URL = "http://agx_orin1:8080"


@app.route('/', methods=['GET'])
def proxy_root():
    """
    Proxy the root `/` to the Web Video Server.
    """
    target_url = WEB_VIDEO_SERVER_BASE_URL
    print(f"Proxying request to: {target_url}")

    try:
        proxied_response = requests.get(
            target_url,
            headers={key: value for (key, value) in request.headers if key.lower() != 'host'},
            params=request.args,
        )
        return Response(proxied_response.content, status=proxied_response.status_code, content_type=proxied_response.headers.get('Content-Type'))
    except requests.exceptions.RequestException as e:
        print(f"Error proxying request to {target_url}: {str(e)}")
        return f"Error: {str(e)}", 500


@app.route('/stream_viewer', methods=['GET'], strict_slashes=False)
def proxy_stream_viewer():
    """
    Proxy the `/stream_viewer` endpoint to the Web Video Server and rewrite links.
    """
    target_url = f"{WEB_VIDEO_SERVER_BASE_URL}/stream_viewer"
    print(f"Proxying request to: {target_url}")
    print(f"Query parameters: {request.args}")

    try:
        # Forward the request to the Web Video Server
        proxied_response = requests.get(
            target_url,
            headers={key: value for (key, value) in request.headers if key.lower() != 'host'},
            params=request.args,
        )
        if 'text/html' in proxied_response.headers.get('Content-Type', ''):
            html_content = proxied_response.text

            # Rewrite the <img src> tag in the HTML to point to the proxy's `/camera/stream`
            html_content = html_content.replace(
                'src="/stream?',
                'src="/camera/stream?'
            )
            print(f"Rewritten HTML: {html_content[:500]}")  # Log the first 500 characters of rewritten HTML
            return Response(html_content, status=proxied_response.status_code, content_type='text/html')

        # Return non-HTML content as-is
        return Response(
            proxied_response.content,
            status=proxied_response.status_code,
            content_type=proxied_response.headers.get('Content-Type'),
        )
    except requests.exceptions.RequestException as e:
        print(f"Error proxying request to {target_url}: {str(e)}")
        return f"Error proxying request to {target_url}: {str(e)}", 500


from urllib.parse import unquote

@app.route('/camera/stream', methods=['GET'])
def proxy_stream():
    """
    Proxy the `/stream` endpoint for MJPEG streaming.
    """
    target_url = f"{WEB_VIDEO_SERVER_BASE_URL}/stream"
    topic = request.args.get('topic', '')
    decoded_topic = unquote(topic)

    print(f"Proxying streaming request to: {target_url}")
    print(f"Decoded topic: {decoded_topic}")

    try:
        # Forward the MJPEG stream request
        proxied_response = requests.get(
            'http://agx_orin1:8080/stream_viewer',
            params={'topic': decoded_topic},
            stream=True,
        )

        # Ensure the response is MJPEG
        content_type = proxied_response.headers.get('Content-Type', '')
        if 'multipart/x-mixed-replace' not in content_type:
            print(f"Unexpected content type: {content_type}")
            return f"Unexpected content type: {content_type}", 500

        # Stream the MJPEG response to the client
        def generate():
            for chunk in proxied_response.iter_content(chunk_size=8192):
                if chunk:
                    yield chunk

        # Return the streamed response with the correct content type
        return Response(
            generate(),
            status=proxied_response.status_code,
            content_type=content_type,
        )
    except requests.exceptions.RequestException as e:
        print(f"Error proxying request to {target_url}: {str(e)}")
        return f"Error proxying request to {target_url}: {str(e)}", 500



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
