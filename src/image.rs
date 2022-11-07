use image::{DynamicImage, io::Reader as ImageReader};

pub fn read_image_from_file<P>(path: P) -> Result<DynamicImage, ReadImageFromFileError> where P: AsRef<std::path::Path> {
    match ImageReader::open(path) {
        Ok(reader) => match reader.decode() {
            Ok(img) => Ok(img),
            Err(e) => Err(ReadImageFromFileError::FailedToDecodeImage(e)),
        },
        Err(e) => Err(ReadImageFromFileError::FailedToOpenFile(e)),
    }
}
pub enum ReadImageFromFileError {
    FailedToOpenFile(std::io::Error),
    FailedToDecodeImage(image::ImageError),
}