use core::utils::{midpoint, Axis, Corner1 as C1, Corner3 as C3, P3, R3, V3};
use core::{
    chain, chain_loop, Dot, DotSpec, MapDots, MinMaxCoord, Shape, Snake, Tree,
};

use errors::ScadDotsError;

#[derive(Debug, Clone, Copy, MapDots, MinMaxCoord, Default)]
pub struct Post {
    pub top: Dot,
    pub bot: Dot,
}

#[derive(Debug, Clone, Copy)]
pub struct PostSpec {
    pub pos: P3,
    pub align: PostAlign,
    pub len: f32,
    pub rot: R3,
    pub size: f32,
    pub shapes: PostShapes,
}

pub trait PostSpecTrait: Copy {
    fn to_dot(&self, upper_or_lower: C1) -> Result<Dot, ScadDotsError>;
}

#[derive(Debug, Copy, Clone)]
pub enum PostAlign {
    Corner {
        post: C1,
        dot: C3,
    },
    Midpoint {
        post_a: C1,
        dot_a: C3,
        post_b: C1,
        dot_b: C3,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PostShapes {
    Cube,
    Sphere,
    Cylinder,
    Round, // bottom cylinder, top sphere
    Custom { top: Shape, bot: Shape },
}

#[derive(Debug, Clone, Copy)]
pub enum PostLink {
    Solid,
    Dots,
}

#[derive(Debug, Clone, Copy, MapDots, MinMaxCoord)]
pub struct PostSnake {
    posts: [Post; 4],
}

#[derive(Debug, Clone, Copy)]
pub enum PostSnakeLink {
    Chain,
    Posts,
}

////////////////////////////////////////////////////////////////////////////////

impl Post {
    /// Create a new Post from the given specification.
    pub fn new<T>(spec: T) -> Result<Post, ScadDotsError>
    where
        T: PostSpecTrait,
    {
        let bot = spec.to_dot(C1::P0)?;
        let top = spec.to_dot(C1::P1)?;

        // TODO fix dimension checking, also dist() changed
        // if bot.dist(top) < size {
        //     bail!("post is too short, top and bottom dot overlap");
        // }
        Ok(Post { top: top, bot: bot })
    }

    /// Return the absolute position of the given alignment point on the Post.
    pub fn pos(&self, align: PostAlign) -> P3 {
        match align {
            PostAlign::Corner { post, dot } => self.pos_corner(post, dot),
            PostAlign::Midpoint {
                post_a,
                dot_a,
                post_b,
                dot_b,
            } => midpoint(
                self.pos_corner(post_a, dot_a),
                self.pos_corner(post_b, dot_b),
            ),
        }
    }

    fn pos_corner(&self, post: C1, dot: C3) -> P3 {
        self.dot(post).pos(dot)
    }

    /// Return a copy of the dot at the upper or lower end of the post.
    pub fn dot(&self, upper_or_lower: C1) -> Dot {
        match upper_or_lower {
            C1::P0 => self.bot,
            C1::P1 => self.top,
        }
    }

    /// Return a vector describing the direction and length of 1 outer edge of
    /// the Post (starting from the Post's origin). The edge's axis is relative
    /// to the Post's default orientation, not it's actual rotated orientation.
    pub fn edge(&self, axis: Axis) -> V3 {
        let origin = self.pos(PostAlign::origin());
        let corner_point = self.pos(PostAlign::outside(axis.into()));
        corner_point - origin
    }

    /// Like `edge()`, but normalizes the vector's length to 1;
    pub fn edge_unit_vec(&self, axis: Axis) -> V3 {
        self.edge(axis).normalize()
    }

    /// Like `edge()`, but returns only the length of the edge.
    pub fn edge_length(&self, axis: Axis) -> f32 {
        self.edge(axis).norm()
    }

    /// Return the size of the Post's Dots.
    pub fn size(&self) -> f32 {
        self.top.size
    }

    /// Make a copy of this Post, but with the lower Dot raised up by the given distance.
    pub fn copy_raise_bot(&self, distance: f32) -> Result<Post, ScadDotsError> {
        if distance > self.edge_length(Axis::Z) - self.top.size {
            return Err(ScadDotsError::Dimension.context(
                "failed to copy_raise_bot, new post would be too short",
            ));
        }
        let translation_vec = distance * self.edge_unit_vec(Axis::Z);
        Ok(Post {
            top: self.top,
            bot: self.bot.translate(translation_vec),
        })
    }

    pub fn snake(
        &self,
        other: Post,
        order: [Axis; 3],
    ) -> Result<PostSnake, ScadDotsError> {
        let tops = Snake::new(self.top, other.top, order)?.dots;
        let bots = Snake::new(self.bot, other.bot, order)?.dots;
        let mut posts = [Post::default(); 4];
        posts[0] = Post {
            top: tops[0],
            bot: bots[0],
        };
        posts[1] = Post {
            top: tops[1],
            bot: bots[1],
        };
        posts[2] = Post {
            top: tops[2],
            bot: bots[2],
        };
        posts[3] = Post {
            top: tops[3],
            bot: bots[3],
        };
        Ok(PostSnake { posts: posts })
    }

    pub fn chain(posts: &[Post]) -> Result<Tree, ScadDotsError> {
        let post_trees: Vec<_> =
            posts.into_iter().map(|p| p.link(PostLink::Solid)).collect();
        chain(&post_trees)
    }

    pub fn chain_loop(posts: &[Post]) -> Result<Tree, ScadDotsError> {
        let post_trees: Vec<_> =
            posts.into_iter().map(|p| p.link(PostLink::Solid)).collect();
        chain_loop(&post_trees)
    }

    pub fn link(&self, style: PostLink) -> Tree {
        match style {
            PostLink::Solid => hull![self.bot, self.top],
            PostLink::Dots => union![self.bot, self.top],
        }
    }
}

impl PostSpecTrait for PostSpec {
    fn to_dot(&self, upper_or_lower: C1) -> Result<Dot, ScadDotsError> {
        let origin =
            self.pos
                - self.align.offset(self.size, self.len - self.size, self.rot);

        let pos =
            origin + upper_or_lower.offset(self.len - self.size, self.rot);
        let spec = DotSpec {
            pos: pos,
            align: C3::P000.into(),
            size: self.size,
            rot: self.rot,
        };
        Ok(Dot::new(self.shapes.get(upper_or_lower), spec))
    }
}

impl PostAlign {
    // TODO add centroid, center_face
    pub fn origin() -> PostAlign {
        PostAlign::outside(C3::P000)
    }

    pub fn outside(corner: C3) -> PostAlign {
        PostAlign::Corner {
            dot: corner,
            post: corner.into(),
        }
    }

    pub fn outside_midpoint(a: C3, b: C3) -> PostAlign {
        PostAlign::midpoint(PostAlign::outside(a), PostAlign::outside(b))
            .expect("bug in outside_midpoint()")
    }

    pub fn midpoint(
        a: PostAlign,
        b: PostAlign,
    ) -> Result<PostAlign, ScadDotsError> {
        match (a, b) {
            (
                PostAlign::Corner {
                    post: post_a,
                    dot: dot_a,
                },
                PostAlign::Corner {
                    post: post_b,
                    dot: dot_b,
                },
            ) => Ok(PostAlign::Midpoint {
                post_a: post_a,
                dot_a: dot_a,
                post_b: post_b,
                dot_b: dot_b,
            }),
            _ => return Err(ScadDotsError::Midpoint),
        }
    }

    fn offset(&self, dot_size: f32, post_length: f32, rot: R3) -> V3 {
        let helper = |post: C1, dot: C3| {
            let dot_spec = dot_size * V3::new(1., 1., 1.);
            dot.offset(dot_spec, rot) + post.offset(post_length, rot)
        };

        match *self {
            PostAlign::Corner { post, dot } => helper(post, dot),
            PostAlign::Midpoint {
                post_a,
                dot_a,
                post_b,
                dot_b,
            } => (helper(post_a, dot_a) + helper(post_b, dot_b)) / 2.,
        }
    }
}

impl PostShapes {
    fn get(&self, upper_or_lower: C1) -> Shape {
        match *self {
            PostShapes::Custom { bot, top } => match upper_or_lower {
                C1::P0 => bot,
                C1::P1 => top,
            },
            PostShapes::Round => match upper_or_lower {
                C1::P0 => Shape::Cylinder,
                C1::P1 => Shape::Sphere,
            },
            PostShapes::Cube => Shape::Cube,
            PostShapes::Sphere => Shape::Sphere,
            PostShapes::Cylinder => Shape::Cylinder,
        }
    }
}

impl PostSnake {
    pub fn bottoms(&self) -> [Dot; 4] {
        [
            self.posts[0].bot,
            self.posts[1].bot,
            self.posts[2].bot,
            self.posts[3].bot,
        ]
    }

    pub fn all(&self) -> [Post; 4] {
        self.posts
    }

    pub fn as_vec(&self) -> Vec<Post> {
        let v: Vec<_> = self.posts.to_vec();
        v
    }

    pub fn get(&self, index: usize) -> Post {
        if let Some(post) = self.posts.get(index).cloned() {
            post
        } else {
            panic!("invalid PostSnake index");
        }
    }

    pub fn link(&self, style: PostSnakeLink) -> Result<Tree, ScadDotsError> {
        match style {
            PostSnakeLink::Chain => Post::chain(&self.posts),
            PostSnakeLink::Posts => {
                // Ok(union![self.posts[0], self.posts[1], self.posts[2], self.posts[2]])
                let v: Vec<_> = self
                    .posts
                    .iter()
                    .map(|post| post.link(PostLink::Solid))
                    .collect();
                Ok(Tree::union(v))
            }
        }
    }
}
