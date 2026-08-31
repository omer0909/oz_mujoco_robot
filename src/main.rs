use rusty_mujoco::{mj_loadXML, mj_makeData, mj_name2id, mj_step, mjr_render, mjv_updateScene};
use rusty_mujoco::{mjData, mjModel};
use rusty_mujoco::{mjrContext, mjrRect, mjtCatBit, mjtFontScale, mjvCamera, mjvOption, mjvScene};

struct Sim {
    pub model: mjModel,
    pub data: mjData,
    glfw: glfw::Glfw,
    window: glfw::PWindow,
    option: mjvOption,
    camera: mjvCamera,
    context: mjrContext,
    scene: mjvScene,
}

impl Sim {
    fn new(xml_path: &str) -> Self {
        let model = mj_loadXML(xml_path).expect("Failed to load XML file");
        let data = mj_makeData(&model);

        let mut glfw = glfw::init(glfw::fail_on_errors).expect("Failed to initialize GLFW");
        glfw.window_hint(glfw::WindowHint::Samples(Some(4)));
        let (mut window, _) = glfw
            .create_window(1200, 900, "Acrobot Simulation", glfw::WindowMode::Windowed)
            .expect("Failed to create GLFW window");
        window.set_size_polling(true);
        window.set_close_polling(true);
        glfw::Context::make_current(&mut *window);

        let context = mjrContext::new(&model, mjtFontScale::X150);
        let option = mjvOption::default();
        let scene = mjvScene::new(&model, 2000);
        let mut camera = mjvCamera::default();

        let camera_name: Option<String> = None;
        camera_name.map(|name| {
            camera.set_fixedcamid(
                mj_name2id(&model, &name).expect("No camera of such name in the model"),
            )
        });

        Sim {
            model,
            data,
            glfw,
            window,
            option,
            camera,
            context,
            scene,
        }
    }

    fn run(&mut self, mut update: impl FnMut(&mut Self)) {
        while !self.window.should_close() {
            while self.data.time() < self.glfw.get_time() {
                mj_step(&self.model, &mut self.data);
                update(self);
            }

            let viewport = {
                let (width, height) = self.window.get_framebuffer_size();
                mjrRect::new(0, 0, width as u32, height as u32)
            };

            mjv_updateScene(
                &self.model,
                &mut self.data,
                &self.option,
                None, /* No perturbation */
                &mut self.camera,
                mjtCatBit::ALL,
                &mut self.scene,
            );
            mjr_render(viewport, &mut self.scene, &self.context);

            glfw::Context::swap_buffers(&mut *self.window);
            self.glfw.poll_events();
        }
    }

    fn lidar_2d_ray(&mut self, count: usize, body: &str) -> Vec<f64> {
        use rusty_mujoco::obj::Body;
        use std::f64::consts::TAU;

        let body_id = mj_name2id::<Body>(&self.model, body).expect("lidar link (body) not found");

        let mut origin = [0.0; 3];
        let mut xmat = [0.0; 9];

        rusty_mujoco::mj_local2Global(
            &mut self.data,
            &mut origin,
            &mut xmat,
            &[0.0, 0.0, 0.0],
            &[1.0, 0.0, 0.0, 0.0],
            body_id,
            rusty_mujoco::bindgen::mjtSameFrame::NONE,
        );

        (0..count)
            .map(|i| {
                let angle = (i as f64) / (count as f64) * TAU;
                let local_dir = [angle.cos(), angle.sin(), 0.0];

                // println!("origin: {:?}", origin);
                // println!("xmat: {:?}", xmat);
                // println!("angle: {:?}", angle);
                // println!("local_dir: {:?}", local_dir);

                let world_dir = [
                    xmat[0] * local_dir[0] + xmat[1] * local_dir[1] + xmat[2] * local_dir[2],
                    xmat[3] * local_dir[0] + xmat[4] * local_dir[1] + xmat[5] * local_dir[2],
                    xmat[6] * local_dir[0] + xmat[7] * local_dir[1] + xmat[8] * local_dir[2],
                ];

                // println!("world_dir: {:?}", world_dir);

                rusty_mujoco::mj_ray(
                    &self.model,
                    &self.data,
                    origin,
                    world_dir,
                    None,
                    true,
                    body_id,
                )
                .map_or(f64::INFINITY, |(_, distance)| distance)
            })
            .collect()
    }
}

fn main() {
    let mut sim = Sim::new("robot.xml");

    let left_motor = mj_name2id::<rusty_mujoco::obj::Actuator>(&sim.model, "wheel_left_motor")
        .expect("actuator not found");
    let right_motor = mj_name2id::<rusty_mujoco::obj::Actuator>(&sim.model, "wheel_right_motor")
        .expect("actuator not found");

    sim.run(move |sim| {
        let rays: Vec<f64> = sim
            .lidar_2d_ray(400, "lidar")
            .into_iter()
            .map(|distance| if distance < 1.0 { f64::NAN } else { distance })
            .collect();

        println!("{:?}", rays);

        sim.data.set_ctrl(left_motor, -1.0);
        sim.data.set_ctrl(right_motor, 1.0);
    });
}
