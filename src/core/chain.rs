use core::utils::Axis;

// #[macro_use(union, hull, mirror)]
// use core;

use core::tree::Tree;
use core::Dot;
use errors::ScadDotsError;
use std::collections::HashSet;

/// Draw a taxicab path between two dots
#[derive(Debug)]
pub struct Snake {
    pub dots: [Dot; 4],
}

#[derive(Debug, Clone, Copy)]
pub enum SnakeLink {
    Chain,
}

impl Snake {
    pub fn new(
        start: Dot,
        end: Dot,
        order: [Axis; 3],
    ) -> Result<Self, ScadDotsError> {
        if Self::has_repeated_axes(order) {
            return Err(ScadDotsError::Snake);
        }
        let mut dots = [Dot::default(); 4];
        dots[0] = start.to_owned();

        // Some dots in the snake may be redundant, and have the same positions.
        // That seems fine.
        for (index, axis) in order.iter().enumerate() {
            dots[index + 1] = dots[index].copy_to_other_dim(end, *axis);
        }
        Ok(Self { dots })
    }

    fn has_repeated_axes(order: [Axis; 3]) -> bool {
        let set: HashSet<_> = order.iter().collect();
        set.len() != order.len()
    }

    pub fn link(&self, style: SnakeLink) -> Result<Tree, ScadDotsError> {
        match style {
            SnakeLink::Chain => chain(&self.dots),
        }
    }
}

/// Store links between each subsequent pair of things
pub fn chain<T>(things: &[T]) -> Result<Tree, ScadDotsError>
where
    T: Clone + Into<Tree>,
{
    let segments: Vec<_> = chain_helper(things)?
        .into_iter()
        .map(|(a, b)| Tree::hull(vec![a, b]))
        .collect();
    Ok(Tree::union(segments))
}

pub fn chain_loop<T>(things: &[T]) -> Result<Tree, ScadDotsError>
where
    T: Clone + Into<Tree>,
{
    let mut circular = things.to_owned();
    circular.push(things.get(0).expect("tried to loop empty slice").to_owned());
    chain(&circular)
}

fn chain_helper<T>(v: &[T]) -> Result<Vec<(T, T)>, ScadDotsError>
where
    T: Clone,
{
    let mut i = v.iter().peekable();
    let mut pairs = Vec::new();
    loop {
        let current = match i.next() {
            Some(n) => n.to_owned(),
            None => return Err(ScadDotsError::Chain),
        };
        let next = match i.peek() {
            Some(&n) => n.to_owned(),
            None => break,
        };
        pairs.push((current.to_owned(), next.to_owned()));
    }
    Ok(pairs)
}
