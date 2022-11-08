extern crate ev3dev_lang_rust;
extern crate image;
extern crate imageproc;

use ev3dev_lang_rust::Screen;

use image::GenericImageView;

// Screen: 178 x 128

// Aufschauklungsverhinderungsalgorithmus

mod roboter;
#[cfg(feature="pc_test")]
mod test;
mod imagehelper;

fn main() {
    let mut screen = Screen::new();
    let mut image_keeper = imagehelper::ImageKeeper::new();
    loop {
        let mut robot = None;
        #[cfg(feature="pc_test")]
        let mut info;
        #[cfg(feature="pc_test")] {
            let img = image::DynamicImage::ImageRgba8(
                crate::test::load_image::load_img("assets/world.png".into()).expect("Failed to load world.png").to_rgba8()
            );
            info = crate::test::virt_info::VirtInfo {
                world_img: img,
                radstand: 0.02, // 20cm
                achsenlänge: 0.01, // 10cm
                color_sensor_distance_from_rotation_point: 0.002, // 2cm
                distance_sensor_pos: ((0.02, 0.0), 0.0),
                color_sensor_radius: 2, // pixels
                max_speed: 0.001,
            };
            robot = Some(roboter::Robot::new(info.clone()));
        }
        #[cfg(not(feature="pc_test"))] {
            robot = Some(roboter::Robot::new());
        }
        let mut robot = robot.unwrap();
        #[cfg(feature="pc_test")] {
            let sender = robot.pc_test_thread.1.clone();
            std::thread::spawn(move || main_ev3(robot).unwrap());
            test::gui::main(sender, info);
        }
        #[cfg(not(feature="pc_test"))] {
            match main_ev3(robot) {
                Ok(_) => {},
                Err(e) => {
                    if let Ok(screen) = &mut screen {
                        let img = match &e {
                            CustomError::FailedToInitializeLinienfolger(e) => match e {
                                roboter::linienfolger::InitError::MissingDevice(dev) => image_keeper.get_err_missing_dev(&dev.into()),
                            },
                            CustomError::FailedToInitializeErkennung(e) => match e {
                                roboter::erkennung::InitError::MissingDevice(dev) => image_keeper.get_err_missing_dev(&dev.into()),
                                roboter::erkennung::InitError::LinienfolgerUnavailable => image_keeper.image_err_missing_ultraschallsensor.get(),
                            },
                        };
                        if let Some(img) = img {
                            for (x, y, px) in img.to_rgb8().enumerate_pixels() {
                                screen.image.put_pixel(x, y, *px);
                            }
                            screen.update();
                        }
                    }
                    println!("Error initializing the program:\n - - - - -\n{:?}\n - - - - -\nExiting in 5 seconds.", e);
                    std::thread::sleep(std::time::Duration::from_secs(5));
                    return;
                },
            };
        }
        break; // unless continue was called, break out of the loop.
    }
}

fn main_ev3(mut robot: roboter::Robot) -> Result<(), CustomError> {
    let sender = robot.thread_linienfolger()?;
    robot.thread_erkennung()?;
    std::thread::spawn(move || {
        std::io::stdin().read_line(&mut String::new()).ok();
        println!("Sending STOP signal...");
        sender.send(roboter::linienfolger::Task::Stop).expect("The other thread is not running.");
    });
    while 0 < robot.check_threads().2 { std::thread::sleep(std::time::Duration::from_secs_f64(0.25)); }
    println!("Threads have stopped.");
    std::thread::sleep(std::time::Duration::from_secs_f64(0.1));
    println!("Exiting.");
    Ok(())
}

#[derive(Debug)]
pub enum CustomError {
    FailedToInitializeLinienfolger(crate::roboter::linienfolger::InitError),
    FailedToInitializeErkennung(crate::roboter::erkennung::InitError),
}

impl From<roboter::linienfolger::InitError> for CustomError { fn from(e: roboter::linienfolger::InitError) -> Self { Self::FailedToInitializeLinienfolger(e) } }
impl From<roboter::erkennung::InitError> for CustomError { fn from(e: roboter::erkennung::InitError) -> Self { Self::FailedToInitializeErkennung(e) } }