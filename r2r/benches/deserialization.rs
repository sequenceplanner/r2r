use criterion::{criterion_group, criterion_main, Criterion};
use r2r::*;
use rand::{thread_rng, Rng};

const NUM_ELEMENTS: usize = 10_000;
const NUM_TIMES: usize = 1_000;

fn bench_ros_deserialization() {
    let mut rng = thread_rng();
    let mut numbers = Vec::<i32>::with_capacity(NUM_ELEMENTS);

    for _ in 0..NUM_ELEMENTS {
        numbers.push(rng.gen_range(0..i32::MAX));
    }
    let message = std_msgs::msg::Int32MultiArray {
        layout: std_msgs::msg::MultiArrayLayout::default(),
        data: numbers,
    };

    let bytes = message.to_serialized_bytes().unwrap();

    for _ in 0..NUM_TIMES {
        let _ = std_msgs::msg::Int32MultiArray::from_serialized_bytes(&bytes).unwrap();
    }
}

fn bench_cdr_deserialization() {
    let mut rng = thread_rng();
    let mut numbers = Vec::<i32>::with_capacity(NUM_ELEMENTS);

    for _ in 0..NUM_ELEMENTS {
        numbers.push(rng.gen_range(0..i32::MAX));
    }
    let message = std_msgs::msg::Int32MultiArray {
        layout: std_msgs::msg::MultiArrayLayout::default(),
        data: numbers,
    };

    let bytes = message.to_serialized_bytes().unwrap();

    for _ in 0..NUM_TIMES {
        let _msg1 = cdr::deserialize::<std_msgs::msg::Int32MultiArray>(&bytes).unwrap();
        // just for testing that we get the same result.
        // let msg2 = std_msgs::msg::Int32MultiArray::from_serialized_bytes(&bytes).unwrap();
        // assert_eq!(msg1, msg2);
        // assert_eq!(msg2, message);
    }
}

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("ros_deserialization", |b| b.iter(|| bench_ros_deserialization()));
    c.bench_function("cdr_deserialization", |b| b.iter(|| bench_cdr_deserialization()));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
