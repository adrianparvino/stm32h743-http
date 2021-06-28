use numtoa::NumToA;

pub struct ResponseBuilder<'a> {
    buf: &'a mut [u8],
    offset: usize
}

impl<'a> ResponseBuilder<'a> {
    pub fn new(buf: &'a mut [u8]) -> ResponseBuilder<'a> {
        ResponseBuilder { buf, offset: 0 }
    }

    pub fn finalize(mut self: ResponseBuilder<'a>) -> &'a [u8] {
        let crlf = b"\r\n";
        self.buf[self.offset..][..crlf.len()].copy_from_slice(crlf);
        self.offset += crlf.len();

        &self.buf[..self.offset]
    }

    pub fn status(mut self, status: u16) -> Self {
        let response = match status {
            200 => b"HTTP/1.1 200 OK \r\n" as &[u8],
            204 => b"HTTP/1.1 204 No Content\r\n" as &[u8],
            404 => b"HTTP/1.1 404 Not Found\r\n" as &[u8],
            _ => panic!()
        };

        self.buf[self.offset..][..response.len()].copy_from_slice(response);
        self.offset += response.len();

        self
    }

    pub fn raw(mut self, line: &[u8]) -> Self {
        self.buf[self.offset..][..line.len()].copy_from_slice(line);
        self.offset += line.len();

        self
    }

    pub fn content_length(mut self, length: usize) -> Self {
        let content_length = b"Content-Length: ";
        self.buf[self.offset..][..content_length.len()].copy_from_slice(content_length);
        self.offset += content_length.len();

        let mut buffer = [0; 16];
        let length_str = length.numtoa(10, &mut buffer);
        self.buf[self.offset..][..length_str.len()].copy_from_slice(length_str);
        self.offset += length_str.len();

        let crlf = b"\r\n";
        self.buf[self.offset..][..crlf.len()].copy_from_slice(crlf);
        self.offset += crlf.len();

        self
    }

    pub fn access_control_allow_headers(mut self, allowed_headers: &str) -> Self {
        let acah = b"Access-Control-Allow-Headers: ";
        self.buf[self.offset..][..acah.len()].copy_from_slice(acah);
        self.offset += acah.len();

        self.buf[self.offset..][..allowed_headers.len()].copy_from_slice(allowed_headers.as_bytes());
        self.offset += allowed_headers.len();

        let crlf = b"\r\n";
        self.buf[self.offset..][..crlf.len()].copy_from_slice(crlf);
        self.offset += crlf.len();

        self
    }

    pub fn access_control_allow_origin(mut self, allowed_origin: &str) -> Self {
        let acao = b"Access-Control-Allow-Origin: ";
        self.buf[self.offset..][..acao.len()].copy_from_slice(acao);
        self.offset += acao.len();

        self.buf[self.offset..][..allowed_origin.len()].copy_from_slice(allowed_origin.as_bytes());
        self.offset += allowed_origin.len();

        let crlf = b"\r\n";
        self.buf[self.offset..][..crlf.len()].copy_from_slice(crlf);
        self.offset += crlf.len();

        self
    }
    pub fn content_encoding(mut self, encoding: &str) -> Self {
        let ce = b"Content-Encoding: ";
        self.buf[self.offset..][..ce.len()].copy_from_slice(ce);
        self.offset += ce.len();

        self.buf[self.offset..][..encoding.len()].copy_from_slice(encoding.as_bytes());
        self.offset += encoding.len();

        let crlf = b"\r\n";
        self.buf[self.offset..][..crlf.len()].copy_from_slice(crlf);
        self.offset += crlf.len();

        self
    }
}
