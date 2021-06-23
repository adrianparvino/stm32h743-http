use core_io::Write;
use core_io::Cursor;

pub struct ResponseBuilder<'a> {
    cursor: Cursor<&'a mut [u8]>,
}

impl<'a> ResponseBuilder<'a> {
    pub fn new(buf: &'a mut [u8]) -> ResponseBuilder<'a> {
        ResponseBuilder { cursor: Cursor::new(buf) }
    }

    pub fn finalize(mut self: ResponseBuilder<'a>) -> &'a [u8] {
        write!(self.cursor, "\r\n").unwrap();

        let len = self.cursor.position() as usize;
        &self.cursor.into_inner()[..len]
    }

    pub fn status(mut self, status: u16) -> Self {
        let response = match status {
            200 => "200 OK",
            204 => "204 No Content",
            404 => "404 Not Found",
            _ => ""
        };

        write!(self.cursor, "HTTP/1.1 {}\r\n", response).unwrap();

        self
    }

    pub fn content_length(mut self, length: usize) -> Self {
        write!(self.cursor, "Content-Length: {}\r\n", length).unwrap();

        self
    }

    pub fn access_control_allow_headers(mut self, allowed_headers: &str) -> Self {
        write!(self.cursor, "Access-Control-Allow-Headers: {}\r\n", allowed_headers).unwrap();

        self
    }

    pub fn access_control_allow_origin(mut self, allowed_origin: &str) -> Self {
        write!(self.cursor, "Access-Control-Allow-Origin: {}\r\n", allowed_origin).unwrap();

        self
    }
    pub fn content_encoding(mut self, encoding: &str) -> Self {
        write!(self.cursor, "Content-Encoding: {}\r\n", encoding).unwrap();

        self
    }
}
