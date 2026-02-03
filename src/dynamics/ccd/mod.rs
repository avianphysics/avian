//! Contains components and functionality for Continuous Collision Detection.
//!
//! # What Is CCD?
//!
//! Physics simulation is typically done in a discrete manner.
//! At the beginning of each physics frame, the simulation checks for collisions,
//! and if none are found for a given rigid body, it is free to move according to
//! its velocity and the size of the timestep.
//!
//! This generally works well for large or slowly moving objects, but fast and small
//! objects can pass through thin geometry such as walls and triangle meshes.
//! This phenomenon is often called **tunneling**.
//!
//! <svg width="300" height="350" viewBox="0 0 300 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="141" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="275" cy="150" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="25" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M275.707 150.707C276.098 150.317 276.098 149.683 275.707 149.293L269.343 142.929C268.953 142.538 268.319 142.538 267.929 142.929C267.538 143.319 267.538 143.953 267.929 144.343L273.586 150L267.929 155.657C267.538 156.047 267.538 156.681 267.929 157.071C268.319 157.462 268.953 157.462 269.343 157.071L275.707 150.707ZM25 151L275 151L275 149L25 149L25 151Z" fill="#E19664"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Discrete</text>
//! </svg>
//!
//! **Continuous Collision Detection** (CCD) aims to prevent tunneling.
//! Currently, two approaches are supported: [Speculative Collision](#speculative-collision) and [Swept CCD](#swept-ccd).
//! Speculative collision is enabled by default, but swept CCD is opt-in with the [`SweptCcd`] component.
//! The two approaches can also be used together.
//!
//! ## Speculative Collision
//!
//! **Speculative collision** is a form of Continuous Collision Detection
//! where contacts are predicted before they happen. The contacts are only
//! solved if the entities are expected to come into contact within the next frame.
//!
//! To determine whether two bodies may come into contact, their [AABBs](ColliderAabb) are expanded
//! based on their velocities. Additionally, a **speculative margin** is used to determine
//! the maximum distance at which a collision pair can generate speculative contacts.
//!
//! <svg width="335" height="400" viewBox="0 0 335 400" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="36" y="266" width="298" height="18" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="210" cy="375" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="210" cy="240" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="160" cy="160" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M209.471 375.849C209.94 376.141 210.557 375.997 210.849 375.529L215.606 367.888C215.898 367.42 215.754 366.803 215.286 366.511C214.817 366.219 214.2 366.363 213.908 366.831L209.68 373.623L202.888 369.394C202.42 369.102 201.803 369.246 201.511 369.714C201.219 370.183 201.363 370.8 201.831 371.092L209.471 375.849ZM159.026 160.227L159.472 162.146L161.42 161.693L160.974 159.773L159.026 160.227ZM160.365 165.985L161.258 169.825L163.206 169.372L162.313 165.532L160.365 165.985ZM162.151 173.664L163.044 177.503L164.992 177.05L164.099 173.211L162.151 173.664ZM163.937 181.343L164.83 185.182L166.778 184.729L165.885 180.89L163.937 181.343ZM165.722 189.021L166.615 192.86L168.563 192.407L167.67 188.568L165.722 189.021ZM167.508 196.7L168.401 200.539L170.349 200.086L169.456 196.247L167.508 196.7ZM169.294 204.378L170.187 208.218L172.135 207.765L171.242 203.925L169.294 204.378ZM171.08 212.057L171.972 215.896L173.92 215.443L173.028 211.604L171.08 212.057ZM172.865 219.735L173.758 223.575L175.706 223.122L174.813 219.282L172.865 219.735ZM174.651 227.414L175.544 231.253L177.492 230.8L176.599 226.961L174.651 227.414ZM176.437 235.093L177.33 238.932L179.278 238.479L178.385 234.64L176.437 235.093ZM178.222 242.771L179.115 246.61L181.063 246.157L180.17 242.318L178.222 242.771ZM180.008 250.45L180.901 254.289L182.849 253.836L181.956 249.997L180.008 250.45ZM181.794 258.128L182.687 261.968L184.635 261.515L183.742 257.675L181.794 258.128ZM183.58 265.807L184.472 269.646L186.42 269.193L185.528 265.354L183.58 265.807ZM185.365 273.485L186.258 277.325L188.206 276.872L187.313 273.032L185.365 273.485ZM187.151 281.164L188.044 285.003L189.992 284.55L189.099 280.711L187.151 281.164ZM188.937 288.843L189.83 292.682L191.778 292.229L190.885 288.39L188.937 288.843ZM190.722 296.521L191.615 300.36L193.563 299.907L192.67 296.068L190.722 296.521ZM192.508 304.2L193.401 308.039L195.349 307.586L194.456 303.747L192.508 304.2ZM194.294 311.878L195.187 315.718L197.135 315.265L196.242 311.425L194.294 311.878ZM196.08 319.557L196.972 323.396L198.92 322.943L198.028 319.104L196.08 319.557ZM197.865 327.235L198.758 331.075L200.706 330.622L199.813 326.782L197.865 327.235ZM199.651 334.914L200.544 338.753L202.492 338.3L201.599 334.461L199.651 334.914ZM201.437 342.593L202.33 346.432L204.278 345.979L203.385 342.14L201.437 342.593ZM203.222 350.271L204.115 354.111L206.063 353.657L205.17 349.818L203.222 350.271ZM205.008 357.95L205.901 361.789L207.849 361.336L206.956 357.497L205.008 357.95ZM206.794 365.628L207.687 369.468L209.635 369.015L208.742 365.175L206.794 365.628ZM208.58 373.307L209.026 375.226L210.974 374.773L210.528 372.854L208.58 373.307ZM209.471 375.849C209.94 376.141 210.557 375.997 210.849 375.529L215.606 367.888C215.898 367.42 215.754 366.803 215.286 366.511C214.817 366.219 214.2 366.363 213.908 366.831L209.68 373.623L202.888 369.394C202.42 369.102 201.803 369.246 201.511 369.714C201.219 370.183 201.363 370.8 201.831 371.092L209.471 375.849ZM159.026 160.227L159.472 162.146L161.42 161.693L160.974 159.773L159.026 160.227ZM160.365 165.985L161.258 169.825L163.206 169.372L162.313 165.532L160.365 165.985ZM162.151 173.664L163.044 177.503L164.992 177.05L164.099 173.211L162.151 173.664ZM163.937 181.343L164.83 185.182L166.778 184.729L165.885 180.89L163.937 181.343ZM165.722 189.021L166.615 192.86L168.563 192.407L167.67 188.568L165.722 189.021ZM167.508 196.7L168.401 200.539L170.349 200.086L169.456 196.247L167.508 196.7ZM169.294 204.378L170.187 208.218L172.135 207.765L171.242 203.925L169.294 204.378ZM171.08 212.057L171.972 215.896L173.92 215.443L173.028 211.604L171.08 212.057ZM172.865 219.735L173.758 223.575L175.706 223.122L174.813 219.282L172.865 219.735ZM174.651 227.414L175.544 231.253L177.492 230.8L176.599 226.961L174.651 227.414ZM176.437 235.093L177.33 238.932L179.278 238.479L178.385 234.64L176.437 235.093ZM178.222 242.771L179.115 246.61L181.063 246.157L180.17 242.318L178.222 242.771ZM180.008 250.45L180.901 254.289L182.849 253.836L181.956 249.997L180.008 250.45ZM181.794 258.128L182.687 261.968L184.635 261.515L183.742 257.675L181.794 258.128ZM183.58 265.807L184.472 269.646L186.42 269.193L185.528 265.354L183.58 265.807ZM185.365 273.485L186.258 277.325L188.206 276.872L187.313 273.032L185.365 273.485ZM187.151 281.164L188.044 285.003L189.992 284.55L189.099 280.711L187.151 281.164ZM188.937 288.843L189.83 292.682L191.778 292.229L190.885 288.39L188.937 288.843ZM190.722 296.521L191.615 300.36L193.563 299.907L192.67 296.068L190.722 296.521ZM192.508 304.2L193.401 308.039L195.349 307.586L194.456 303.747L192.508 304.2ZM194.294 311.878L195.187 315.718L197.135 315.265L196.242 311.425L194.294 311.878ZM196.08 319.557L196.972 323.396L198.92 322.943L198.028 319.104L196.08 319.557ZM197.865 327.235L198.758 331.075L200.706 330.622L199.813 326.782L197.865 327.235ZM199.651 334.914L200.544 338.753L202.492 338.3L201.599 334.461L199.651 334.914ZM201.437 342.593L202.33 346.432L204.278 345.979L203.385 342.14L201.437 342.593ZM203.222 350.271L204.115 354.111L206.063 353.657L205.17 349.818L203.222 350.271ZM205.008 357.95L205.901 361.789L207.849 361.336L206.956 357.497L205.008 357.95ZM206.794 365.628L207.687 369.468L209.635 369.015L208.742 365.175L206.794 365.628ZM208.58 373.307L209.026 375.226L210.974 374.773L210.528 372.854L208.58 373.307Z" fill="#AF644B"/>
//!     <path d="M209.775 240.974C210.313 241.099 210.85 240.763 210.974 240.225L212.998 231.455C213.122 230.917 212.787 230.38 212.249 230.256C211.71 230.132 211.174 230.467 211.049 231.006L209.25 238.801L201.455 237.002C200.917 236.878 200.38 237.213 200.256 237.751C200.132 238.29 200.467 238.826 201.006 238.951L209.775 240.974ZM159.152 160.53L209.152 240.53L210.848 239.47L160.848 159.47L159.152 160.53Z" fill="#E19664"/>
//!     <circle cx="160" cy="265" r="4" fill="#a874d8"/>
//!     <path d="M160 160V265" stroke="#a874d8" stroke-width="2" stroke-dasharray="5 5"/>
//!     <rect x="135" y="135" width="99" height="264" stroke="#db9010" stroke-width="1"/>
//!     <circle cx="160" cy="160" r="159" stroke="#eb4a5a" stroke-width="1"/>
//!     <text x="184" y="125" style="fill: #db9010; font: 16px monospace; text-anchor: middle;">Collision AABB</text>
//!     <text x="160" y="40" style="fill: #eb4a5a; font: 16px monospace; text-anchor: middle;">Speculative Margin</text>
//!     <text x="210" y="340" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Unconstrained</text>
//!     <text x="210" y="205" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Constrained</text>
//!     <text y="240" style="fill: #a874d8; font: 16px monospace; font-weight: bold; text-anchor: end;">
//!         <tspan x="155">Speculative</tspan><tspan x="155" dy="18">Contact</tspan>
//!     </text>
//! </svg>
//!
//! The current "effective" speculative margin for a body is determined by its velocity
//! clamped by the specified maximum bound. By default, the maximum bound is infinite,
//! but it can be configured for all entities using the [`NarrowPhaseConfig`] resource,
//! and for individual entities using the [`SpeculativeMargin`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
//!     // Spawn a rigid body with a maximum bound for the speculative margin.
//!     commands.spawn((
//!         RigidBody::Dynamic,
//!         Collider::capsule(0.5, 2.0),
//!         SpeculativeMargin(2.0),
//!     ));
//! }
//! ```
//!
//! Speculative collisions are an efficient and generally robust approach
//! to Continuous Collision Detection. They are enabled for all bodies by default,
//! provided that the default maximum speculative margin is greater than zero.
//!
//! However, speculative collisions aren't entirely without issues,
//! and there are some cases where large speculative margins can cause undesired behavior.
//!
//! ### Caveats of Speculative Collision
//!
//! Speculative contacts are approximations. They typically have good enough accuracy,
//! but when bodies are moving past each other at high speeds, the prediction can sometimes
//! fail and lead to **ghost collisions**. This happens because contact surfaces are treated
//! like infinite planes from the point of view of the solver. Ghost collisions typically manifest
//! as objects bumping into seemingly invisible walls.
//!
//! <svg width="475" height="400" viewBox="0 0 475 400" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="36" y="266" width="148" height="18" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="345" cy="375" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="345" cy="236" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="160" cy="160" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M344.925 375.997C345.476 376.038 345.956 375.626 345.997 375.075L346.67 366.1C346.712 365.549 346.299 365.069 345.748 365.028C345.197 364.987 344.717 365.4 344.676 365.95L344.078 373.928L336.1 373.33C335.549 373.288 335.069 373.701 335.028 374.252C334.987 374.803 335.4 375.283 335.95 375.324L344.925 375.997ZM159.242 160.652L160.563 162.188L162.079 160.883L160.758 159.348L159.242 160.652ZM163.206 165.259L165.849 168.331L167.365 167.026L164.722 163.955L163.206 165.259ZM168.492 171.402L171.135 174.474L172.651 173.169L170.008 170.098L168.492 171.402ZM173.778 177.545L176.421 180.617L177.937 179.312L175.294 176.241L173.778 177.545ZM179.063 183.688L181.706 186.759L183.222 185.455L180.579 182.383L179.063 183.688ZM184.349 189.831L186.992 192.902L188.508 191.598L185.865 188.526L184.349 189.831ZM189.635 195.974L192.278 199.045L193.794 197.741L191.151 194.669L189.635 195.974ZM194.921 202.117L197.563 205.188L199.079 203.883L196.437 200.812L194.921 202.117ZM200.206 208.259L202.849 211.331L204.365 210.026L201.722 206.955L200.206 208.259ZM205.492 214.402L208.135 217.474L209.651 216.169L207.008 213.098L205.492 214.402ZM210.778 220.545L213.421 223.617L214.937 222.312L212.294 219.241L210.778 220.545ZM216.063 226.688L218.706 229.759L220.222 228.455L217.579 225.383L216.063 226.688ZM221.349 232.831L223.992 235.902L225.508 234.598L222.865 231.526L221.349 232.831ZM226.635 238.974L229.278 242.045L230.794 240.741L228.151 237.669L226.635 238.974ZM231.921 245.117L234.563 248.188L236.079 246.883L233.437 243.812L231.921 245.117ZM237.206 251.259L239.849 254.331L241.365 253.026L238.722 249.955L237.206 251.259ZM242.492 257.402L245.135 260.474L246.651 259.169L244.008 256.098L242.492 257.402ZM247.778 263.545L250.421 266.617L251.937 265.312L249.294 262.241L247.778 263.545ZM253.063 269.688L255.706 272.759L257.222 271.455L254.579 268.383L253.063 269.688ZM258.349 275.831L260.992 278.902L262.508 277.598L259.865 274.526L258.349 275.831ZM263.635 281.974L266.278 285.045L267.794 283.741L265.151 280.669L263.635 281.974ZM268.921 288.116L271.563 291.188L273.079 289.883L270.437 286.812L268.921 288.116ZM274.206 294.259L276.849 297.331L278.365 296.026L275.722 292.955L274.206 294.259ZM279.492 300.402L282.135 303.474L283.651 302.169L281.008 299.098L279.492 300.402ZM284.778 306.545L287.421 309.616L288.937 308.312L286.294 305.241L284.778 306.545ZM290.063 312.688L292.706 315.759L294.222 314.455L291.579 311.383L290.063 312.688ZM295.349 318.831L297.992 321.902L299.508 320.598L296.865 317.526L295.349 318.831ZM300.635 324.974L303.278 328.045L304.794 326.741L302.151 323.669L300.635 324.974ZM305.921 331.116L308.563 334.188L310.079 332.883L307.437 329.812L305.921 331.116ZM311.206 337.259L313.849 340.331L315.365 339.026L312.722 335.955L311.206 337.259ZM316.492 343.402L319.135 346.474L320.651 345.169L318.008 342.098L316.492 343.402ZM321.778 349.545L324.421 352.616L325.937 351.312L323.294 348.241L321.778 349.545ZM327.063 355.688L329.706 358.759L331.222 357.455L328.579 354.383L327.063 355.688ZM332.349 361.831L334.992 364.902L336.508 363.598L333.865 360.526L332.349 361.831ZM337.635 367.974L340.278 371.045L341.794 369.741L339.151 366.669L337.635 367.974ZM342.921 374.117L344.242 375.652L345.758 374.348L344.437 372.812L342.921 374.117ZM344.925 375.997C345.476 376.038 345.956 375.626 345.997 375.075L346.67 366.1C346.712 365.549 346.299 365.069 345.748 365.028C345.197 364.987 344.717 365.4 344.676 365.95L344.078 373.928L336.1 373.33C335.549 373.288 335.069 373.701 335.028 374.252C334.987 374.803 335.4 375.283 335.95 375.324L344.925 375.997ZM159.242 160.652L160.563 162.188L162.079 160.883L160.758 159.348L159.242 160.652ZM163.206 165.259L165.849 168.331L167.365 167.026L164.722 163.955L163.206 165.259ZM168.492 171.402L171.135 174.474L172.651 173.169L170.008 170.098L168.492 171.402ZM173.778 177.545L176.421 180.617L177.937 179.312L175.294 176.241L173.778 177.545ZM179.063 183.688L181.706 186.759L183.222 185.455L180.579 182.383L179.063 183.688ZM184.349 189.831L186.992 192.902L188.508 191.598L185.865 188.526L184.349 189.831ZM189.635 195.974L192.278 199.045L193.794 197.741L191.151 194.669L189.635 195.974ZM194.921 202.117L197.563 205.188L199.079 203.883L196.437 200.812L194.921 202.117ZM200.206 208.259L202.849 211.331L204.365 210.026L201.722 206.955L200.206 208.259ZM205.492 214.402L208.135 217.474L209.651 216.169L207.008 213.098L205.492 214.402ZM210.778 220.545L213.421 223.617L214.937 222.312L212.294 219.241L210.778 220.545ZM216.063 226.688L218.706 229.759L220.222 228.455L217.579 225.383L216.063 226.688ZM221.349 232.831L223.992 235.902L225.508 234.598L222.865 231.526L221.349 232.831ZM226.635 238.974L229.278 242.045L230.794 240.741L228.151 237.669L226.635 238.974ZM231.921 245.117L234.563 248.188L236.079 246.883L233.437 243.812L231.921 245.117ZM237.206 251.259L239.849 254.331L241.365 253.026L238.722 249.955L237.206 251.259ZM242.492 257.402L245.135 260.474L246.651 259.169L244.008 256.098L242.492 257.402ZM247.778 263.545L250.421 266.617L251.937 265.312L249.294 262.241L247.778 263.545ZM253.063 269.688L255.706 272.759L257.222 271.455L254.579 268.383L253.063 269.688ZM258.349 275.831L260.992 278.902L262.508 277.598L259.865 274.526L258.349 275.831ZM263.635 281.974L266.278 285.045L267.794 283.741L265.151 280.669L263.635 281.974ZM268.921 288.116L271.563 291.188L273.079 289.883L270.437 286.812L268.921 288.116ZM274.206 294.259L276.849 297.331L278.365 296.026L275.722 292.955L274.206 294.259ZM279.492 300.402L282.135 303.474L283.651 302.169L281.008 299.098L279.492 300.402ZM284.778 306.545L287.421 309.616L288.937 308.312L286.294 305.241L284.778 306.545ZM290.063 312.688L292.706 315.759L294.222 314.455L291.579 311.383L290.063 312.688ZM295.349 318.831L297.992 321.902L299.508 320.598L296.865 317.526L295.349 318.831ZM300.635 324.974L303.278 328.045L304.794 326.741L302.151 323.669L300.635 324.974ZM305.921 331.116L308.563 334.188L310.079 332.883L307.437 329.812L305.921 331.116ZM311.206 337.259L313.849 340.331L315.365 339.026L312.722 335.955L311.206 337.259ZM316.492 343.402L319.135 346.474L320.651 345.169L318.008 342.098L316.492 343.402ZM321.778 349.545L324.421 352.616L325.937 351.312L323.294 348.241L321.778 349.545ZM327.063 355.688L329.706 358.759L331.222 357.455L328.579 354.383L327.063 355.688ZM332.349 361.831L334.992 364.902L336.508 363.598L333.865 360.526L332.349 361.831ZM337.635 367.974L340.278 371.045L341.794 369.741L339.151 366.669L337.635 367.974ZM342.921 374.117L344.242 375.652L345.758 374.348L344.437 372.812L342.921 374.117Z" fill="#AF644B"/>
//!     <path d="M345.385 236.923C345.895 236.71 346.136 236.124 345.923 235.615L342.454 227.31C342.242 226.8 341.656 226.56 341.146 226.772C340.637 226.985 340.396 227.571 340.609 228.08L343.692 235.463L336.31 238.546C335.8 238.758 335.56 239.344 335.772 239.854C335.985 240.363 336.571 240.604 337.08 240.391L345.385 236.923ZM159.62 160.925L344.62 236.925L345.38 235.075L160.38 159.075L159.62 160.925Z" fill="#E19664"/>
//!     <circle cx="160" cy="265" r="4" fill="#a874d8"/>
//!     <path d="M160 160V265" stroke="#a874d8" stroke-width="2" stroke-dasharray="5 5"/>
//!     <rect x="135" y="135" width="235" height="264" stroke="#db9010" stroke-width="1"/>
//!     <circle cx="160" cy="160" r="159" stroke="#eb4a5a" stroke-width="1"/>
//!     <line x1="160" y1="264" x2="400" y2="264" stroke="#a874d8" stroke-width="2" stroke-dasharray="6 6"/>
//!     <text x="184" y="125" style="fill: #db9010; font: 16px monospace; text-anchor: middle;">Collision AABB</text>
//!     <text x="160" y="40" style="fill: #eb4a5a; font: 16px monospace; text-anchor: middle;">Speculative Margin</text>
//!     <text x="345" y="340" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Unconstrained</text>
//!     <text x="345" y="205" style="fill: #b4b4b4; font: 16px monospace; text-anchor: start;">Constrained</text>
//!     <text x="190" y="280" style="fill: #a874d8; font: 16px monospace; font-weight: bold; text-anchor: start;">Ghost Collision Plane</text>
//! </svg>
//!
//! Ghost collisions can be mitigated by using a smaller [`SpeculativeMargin`]
//! or a higher [`Physics`] timestep rate.
//!
//! Another caveat of speculative collisions is that they can still occasionally miss contacts,
//! especially for thin objects spinning at very high speeds. This is typically quite rare however,
//! and speculative collision should work fine for the majority of cases.
//!
//! Speculative collisions can also absorb some energy in contacts, causing even perfectly elastic
//! objects to lose kinetic energy over several bounces.
//!
//! For an approach that is more expensive but doesn't suffer from ghost collisions,
//! missed collisions, or inaccurate restitution, consider using swept CCD,
//! which is described in the following section.
//!
//! ## Swept CCD
//!
//! *Note: Swept CCD currently only supports the built-in `Collider`.*
//!
//! **Swept CCD** is a form of Continuous Collision Detection that sweeps potentially colliding objects
//! from their previous positions to their current positions, and if a collision is found, moves the bodies
//! back to the time of impact. This way, the normal collision algorithms will be able to detect and handle
//! the collision during the next frame.
//!
//! There are two variants of swept CCD: [`Linear`](SweepMode::Linear)
//! and [`NonLinear`](SweepMode::Linear). The difference between the two
//! is that [`Linear`](SweepMode::Linear) only considers translational motion,
//! so bodies can still pass through objects that are spinning at high speeds,
//! while [`NonLinear`](SweepMode::NonLinear) also considers rotational motion,
//! but is more expensive.
//!
//! <svg width="300" height="350" viewBox="0 0 300 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="141" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="115" cy="150" r="24" stroke="#5064C8" stroke-width="2" stroke-dasharray="4 4"/>
//!     <circle cx="25" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <path d="M115.707 150.707C116.098 150.317 116.098 149.683 115.707 149.293L109.343 142.929C108.953 142.538 108.319 142.538 107.929 142.929C107.538 143.319 107.538 143.953 107.929 144.343L113.586 150L107.929 155.657C107.538 156.047 107.538 156.681 107.929 157.071C108.319 157.462 108.953 157.462 109.343 157.071L115.707 150.707ZM25 151H115V149H25V151Z" fill="#E19664"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Linear / NonLinear</text>
//! </svg>
//!
//! <svg width="600" height="350" viewBox="0 0 600 350" fill="none" xmlns="http://www.w3.org/2000/svg">
//!     <rect x="438.95" y="25.3488" width="18" height="298" transform="rotate(27.5 438.95 25.3488)" stroke="#64C850" stroke-width="2" stroke-dasharray="4 4"/>
//!     <rect x="301" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="425" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <text x="150" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">Linear</text>
//!     <circle cx="310" cy="290" r="5" fill="#D9D9D9" stroke="black" stroke-width="2"/>
//!     <path d="M322.96 148.816L323.331 149.745L323.331 149.745L322.96 148.816ZM361.786 156.786L361.078 157.493L361.078 157.493L361.786 156.786ZM365 161C365.552 161 366 160.552 366 160L366 151C366 150.448 365.552 150 365 150C364.448 150 364 150.448 364 151L364 159L356 159C355.448 159 355 159.448 355 160C355 160.552 355.448 161 356 161L365 161ZM320.371 150.928L323.331 149.745L322.588 147.888L319.629 149.072L320.371 150.928ZM361.078 157.493L364.293 160.707L365.707 159.293L362.493 156.078L361.078 157.493ZM323.331 149.745C336.331 144.545 351.178 147.592 361.078 157.493L362.493 156.078C352.027 145.612 336.331 142.391 322.588 147.888L323.331 149.745Z" fill="#E19664"/>
//!     <rect x="259.442" y="133.366" width="18" height="298" transform="rotate(60 259.442 133.366)" stroke="#64C850" stroke-width="2" stroke-dasharray="4 4"/>
//!     <rect x="1" y="1" width="18" height="298" fill="#64C850" stroke="black" stroke-width="2"/>
//!     <circle cx="125" cy="150" r="24" fill="#5064C8" stroke="black" stroke-width="2"/>
//!     <text x="450" y="325" style="fill: #b4b4b4; font: 18px monospace; text-anchor: middle;">NonLinear</text>
//!     <circle cx="10" cy="290" r="5" fill="#D9D9D9" stroke="black" stroke-width="2"/>
//!     <path d="M54.0561 245.357L53.1144 245.694L53.1144 245.694L54.0561 245.357ZM54.5719 248.904C55.071 249.14 55.6673 248.927 55.9037 248.428L59.7565 240.294C59.9929 239.795 59.7799 239.199 59.2808 238.963C58.7817 238.726 58.1854 238.939 57.949 239.438L54.5243 246.668L47.2944 243.243C46.7953 243.007 46.199 243.22 45.9626 243.719C45.7261 244.218 45.9391 244.815 46.4382 245.051L54.5719 248.904ZM53.1144 245.694L54.0582 248.336L55.9417 247.664L54.9978 245.021L53.1144 245.694ZM20.3714 230.928C33.4979 225.678 48.3594 232.379 53.1144 245.694L54.9978 245.021C49.8615 230.639 33.808 223.4 19.6286 229.071L20.3714 230.928Z" fill="#E19664"/>
//! </svg>
//!
//! To enable swept CCD for a rigid body, simply add the [`SweptCcd`] component
//! and make sure that the [`CcdPlugin`] is enabled. The plugin is included
//! in the [`PhysicsPlugins`] plugin group.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! fn setup(mut commands: Commands) {
//!     // Spawn a rigid body with swept CCD enabled.
//!     // `SweepMode::NonLinear` is used by default.
//!     commands.spawn((
//!         RigidBody::Dynamic,
//!         Collider::capsule(0.5, 2.0),
//!         SweptCcd::default(),
//!     ));
//! }
//! ```
//!
//! See the [documentation](SweptCcd) of the component for more details
//! and configuration options.
//!
//! Swept CCD is more expensive than [Speculative Collision](#speculative-collision),
//! but it doesn't have the same issues with ghost collisions or missed collisions.
//! It is primarily intended as a safety net when it is crucial that a body never
//! tunnels through geometry. Swept CCD should only be used when necessary,
//! as it also has its own set of problems.
//!
//! ### Caveats of Swept CCD
//!
//! The [`Linear`](SweepMode::Linear) and [`NonLinear`](SweepMode::Linear)
//! approaches can lead to *time loss* or *time stealing*, where bodies appear to momentarily
//! move slower when Swept CCD is active. This happens because they are essentially moved
//! backwards in time to avoid missing the collision.
//!
//! Swept CCD might also not account for chain reactions properly, so if a fast-moving body
//! bumps into another body, making it a fast-moving body that potentially bumps into
//! even more bodies, the collisions might not propagate accurately.
//! However, speculative contacts do detect secondary collisions quite well,
//! so using the two together can help mitigate the issue.
//!
//! Time loss and chain reactions could also be better accounted for with a substepped
//! time-of-impact solver, but that would be much more expensive and complex,
//! so it is not yet supported.
//!
//! ## Other Ways to Avoid Tunneling
//!
//! CCD is one way to prevent objects from tunneling through each other,
//! but it should only be used when necessary. There are several other approaches
//! worth considering to help avoid the issue.
//!
//! The most obvious way is to simply avoid small or thin geometry such as triangle meshes,
//! and to make colliders for objects like walls slightly thicker than necessary.
//! This is of course typically not possible everywhere, but it is good to keep in mind
//! when authoring levels.
//!
//! Triangle mesh colliders are especially prone to tunneling for dynamic rigid bodies.
//! For shapes that are intended to be solid from the inside, it is recommended
//! to use convex decomposition instead.
//!
//! If you must use triangle mesh colliders and are having stability issues, consider
//! giving them a small amount of extra thickness using the [`CollisionMargin`] component.
//! This helps prevent objects from passing through the surface while also reducing
//! numerical errors and improving performance.
//!
//! Finally, making the [physics timestep](Physics) smaller can also help.
//! However, this comes at the cost of worse performance for the entire simulation.

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use super::solver::solver_body::SolverBody;
use crate::prelude::*;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use bevy::ecs::query::QueryData;
use bevy::prelude::*;
use derive_more::From;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use dynamics::solver::SolverDiagnostics;
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
use parry::query::{
    NonlinearRigidMotion, ShapeCastHit, ShapeCastOptions, cast_shapes, cast_shapes_nonlinear,
};

/// A plugin for [Continuous Collision Detection](self).
pub struct CcdPlugin;

impl Plugin for CcdPlugin {
    fn build(&self, app: &mut App) {
        // Get the `PhysicsSchedule`, and panic if it doesn't exist.
        let physics = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        physics.configure_sets(
            SweptCcdSystems
                .after(SolverSystems::PostSubstep)
                .before(SolverSystems::Restitution),
        );

        #[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
        physics.add_systems(solve_swept_ccd.in_set(SweptCcdSystems));
    }
}

/// A system set for [Continuous Collision Detection](self) systems.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SweptCcdSystems;

/// A deprecated alias for [`SweptCcdSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `SweptCcdSystems`")]
pub type SweptCcdSet = SweptCcdSystems;

/// The maximum distance at which [speculative contacts](self#speculative-collision)
/// are generated for this entity. A value greater than zero helps prevent missing
/// contacts for moderately fast-moving and thin objects.
///
/// In general, this component should not be needed. The default speculative margin
/// for all objects is defined in the [`NarrowPhaseConfig`] resource, and it is unbounded
/// by default, meaning that the margin can extend infinitely. The margin can be bounded
/// for individual entities using this component in case speculative contacts are causing
/// issues like the ones outlined [here](self#caveats-of-speculative-collision).
///
/// See the [module-level documentation](self) for information about
/// Speculative Conllision and Continuous Collision Detection in general.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a rigid body with an unbounded speculative margin.
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 2.0),
///         SpeculativeMargin::MAX,
///     ));
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, Deref, DerefMut, PartialEq, Reflect, From)]
#[reflect(Component)]
#[doc(alias = "SweptCcdPredictionDistance")]
pub struct SpeculativeMargin(pub Scalar);

impl SpeculativeMargin {
    /// A zero speculative margin. Disables speculative collision for this entity.
    pub const ZERO: Self = Self(0.0);

    /// An unbounded speculative margin.
    pub const MAX: Self = Self(Scalar::MAX);
}

/// A component that enables sweep-based [Continuous Collision Detection](self) (CCD)
/// for a [`RigidBody`]. This helps prevent missed collisions for small and fast-moving objects.
///
/// By default, swept CCD considers both translational and rotational motion, and is used
/// against all types of rigid bodies and colliders. This behavior can be modified
/// by changing [`SweptCcd::mode`] and other properties.
///
/// CCD is generally not useful for large or slow objects, as they are less likely
/// to miss collisions. It is recommended to only use swept CCD when necessary,
/// as it is more expensive than [speculative collision](self#speculative-collision) and discrete collision detection.
///
/// Read the [module-level documentation](self) for more information about what CCD is,
/// what it is used for, and what limitations and tradeoffs it can have.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// # #[cfg(feature = "f32")]
/// fn setup(mut commands: Commands) {
///     // Spawn a dynamic rigid body with swept CCD, travelling towards the right at a high speed.
///     // The default CCD configuration considers both translational and rotational motion.
///     commands.spawn((
///         RigidBody::Dynamic,
///         SweptCcd::default(),
#[cfg_attr(feature = "2d", doc = "        LinearVelocity(Vec2::X * 100.0),")]
#[cfg_attr(feature = "3d", doc = "        LinearVelocity(Vec3::X * 100.0),")]
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.1),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.1),")]
///         Transform::from_xyz(-10.0, 3.0, 0.0),
///     ));
///
///     // Spawn another dynamic rigid body with swept CCD, but this time only considering
///     // linear motion and not rotation.
///     commands.spawn((
///         RigidBody::Dynamic,
///         SweptCcd::LINEAR, // or `SweptCcd::new_with_mode(SweepMode::Linear)`
#[cfg_attr(feature = "2d", doc = "        LinearVelocity(Vec2::X * 100.0),")]
#[cfg_attr(feature = "3d", doc = "        LinearVelocity(Vec3::X * 100.0),")]
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.1),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.1),")]
///         Transform::from_xyz(-10.0, -3.0, 0.0),
///     ));
///
///     // Spawn a thin, long object rotating at a high speed.
///     // The first ball should hit it, but the second one does not consider
///     // rotational motion, and most likely passes through.
///     commands.spawn((
///         RigidBody::Dynamic,
///         LockedAxes::TRANSLATION_LOCKED,
#[cfg_attr(feature = "2d", doc = "        AngularVelocity(100.0),")]
#[cfg_attr(feature = "3d", doc = "        AngularVelocity(Vec3::Z * 100.0),")]
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(0.2, 10.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(0.2, 10.0, 10.0),")]
///     ));
///
///     // Spawn another thin, long object, this time not rotating.
///     // The second ball should now hit this.
///     commands.spawn((
///         RigidBody::Static,
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(0.2, 10.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(0.2, 10.0, 10.0),")]
///         Transform::from_xyz(15.0, 0.0, 0.0),
///     ));
/// }
/// ```
#[derive(Component, Clone, Copy, Debug, PartialEq, Reflect)]
#[reflect(Component)]
pub struct SweptCcd {
    /// The type of sweep used for swept CCD.
    ///
    /// If two entities with different sweep modes collide, [`SweepMode::NonLinear`]
    /// is preferred.
    ///
    /// The default is [`SweepMode::NonLinear`], which considers both translational
    /// and rotational motion.
    pub mode: SweepMode,
    /// If `true`, swept CCD is performed against dynamic rigid bodies.
    /// Otherwise, it is only performed against static geometry and kinematic bodies.
    ///
    /// The default is `true`.
    pub include_dynamic: bool,
    /// Determines how fast two bodies must be moving relative to each other
    /// for swept CCD to be activated for them.
    ///
    /// If the linear velocity is below this threshold, CCD is skipped,
    /// unless the angular velocity exceeds the `angular_threshold`.
    ///
    /// The default is `0.0`, meaning that CCD is performed regardless of the relative velocity.
    pub linear_threshold: Scalar,
    /// Determines how fast two bodies must be rotating relative to each other
    /// for swept CCD to be activated for them.
    ///
    /// If the angular velocity is below this threshold, CCD is skipped,
    /// unless the linear velocity exceeds the `linear_threshold`.
    ///
    /// The default is `0.0`, meaning that CCD is performed regardless of the relative velocity.
    pub angular_threshold: Scalar,
}

impl Default for SweptCcd {
    fn default() -> Self {
        Self::NON_LINEAR
    }
}

impl SweptCcd {
    /// Continuous Collision Detection with [`SweepMode::Linear`].
    ///
    /// This only takes into account translational motion, and can lead to tunneling
    /// against thin, fast-spinning objects.
    pub const LINEAR: Self = Self::new_with_mode(SweepMode::Linear);

    /// Continuous Collision Detection with [`SweepMode::NonLinear`].
    ///
    /// This takes into account both translational and rotational motion.
    pub const NON_LINEAR: Self = Self::new_with_mode(SweepMode::NonLinear);

    /// Creates a new [`SweptCcd`] configuration with the given [`SweepMode`].
    #[inline]
    pub const fn new_with_mode(mode: SweepMode) -> Self {
        Self {
            mode,
            include_dynamic: true,
            linear_threshold: 0.0,
            angular_threshold: 0.0,
        }
    }

    /// Sets the linear and angular velocity thresholds in `self`,
    /// determining how fast two bodies must be moving relative to each other
    /// for swept CCD to be activated for them.
    ///
    /// CCD will be active if either of the two thresholds is exceeded.
    #[inline]
    pub const fn with_velocity_threshold(mut self, linear: Scalar, angular: Scalar) -> Self {
        self.linear_threshold = linear;
        self.angular_threshold = angular;
        self
    }

    /// Sets whether swept CCD is performed against dynamic rigid bodies.
    /// If `false`, it is only performed against static geometry and kinematic bodies.
    #[inline]
    pub const fn include_dynamic(mut self, should_include: bool) -> Self {
        self.include_dynamic = should_include;
        self
    }
}

/// The algorithm used for [Swept Continuous Collision Detection](self#swept-ccd).
///
/// If two entities with different sweep modes collide, [`SweepMode::NonLinear`]
/// is preferred.
///
/// The default is [`SweepMode::NonLinear`], which considers both translational
/// and rotational motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum SweepMode {
    /// [Swept CCD](self#swept-ccd) is performed using linear time-of-impact queries
    /// from the previous positions of [`SweptCcd`] bodies to their current positions,
    /// stopping the bodies at the first time of impact.
    ///
    /// This mode only considers translational motion, and can lead to tunneling
    /// against thin, fast-spinning objects. For the more expensive version
    /// that also considers rotational motion, consider using [`SweepMode::NonLinear`].
    Linear,

    /// [Swept CCD](self#swept-ccd) is performed using non-linear time-of-impact queries
    /// from the previous positions of [`SweptCcd`] bodies to their current positions,
    /// stopping the bodies at the first time of impact.
    ///
    /// This mode considers both translational and rotational motion.
    /// For the cheaper version that only considers translational motion,
    /// consider using [`SweepMode::Linear`].
    #[default]
    NonLinear,
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
#[derive(QueryData)]
#[query_data(mutable)]
struct SweptCcdBodyQuery {
    entity: Entity,
    solver_body: Option<&'static mut SolverBody>,
    rb: &'static RigidBody,
    pos: &'static Position,
    rot: &'static Rotation,
    ccd: Option<&'static SweptCcd>,
    collider: &'static Collider,
    com: &'static ComputedCenterOfMass,
}

/// Performs [sweep-based mContinuous Collision Detection](self#swept-ccd)
/// by performing time-of-impact queries from the previous positions of [`SweptCcd`] bodies
/// to their current positions, stopping the bodies at the first time of impact.
///
/// This approach can lead to "time loss" or "time stealing", because the bodies
/// are essentially moved back in time, making them appear to momentarily move slower.
/// Secondary contacts are also not accounted for.
#[allow(clippy::useless_conversion)]
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn solve_swept_ccd(
    ccd_query: Query<Entity, With<SweptCcd>>,
    bodies: Query<SweptCcdBodyQuery>,
    colliders: Query<(&Collider, &ColliderOf)>,
    time: Res<Time>,
    contact_graph: Res<ContactGraph>,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    mut diagnostics: ResMut<SolverDiagnostics>,
) {
    let start = crate::utils::Instant::now();

    let delta_secs = time.delta_seconds_adjusted();

    let mut dummy_body = SolverBody::default();

    // TODO: Parallelize.
    for entity in &ccd_query {
        // Get the CCD body.
        let Ok(SweptCcdBodyQueryItem {
            solver_body: Some(mut solver_body1),
            pos: &prev_pos1,
            rot: &prev_rot1,
            ccd: Some(ccd1),
            collider: collider1,
            com: com1,
            ..
        }) = (
            // Safety: `get_unchecked` is only unsafe if there are multiple mutable references
            //         to the same component, and this is ensured not to happen when iterating
            //         over the AABB intersections below.
            unsafe { bodies.get_unchecked(entity) }
        )
        else {
            continue;
        };

        let lin_vel1 = solver_body1.linear_velocity;
        let ang_vel1 = solver_body1.angular_velocity;

        // The smallest time of impact found. Starts at the largest value,
        // how long a body moves during a single frame due to position integration.
        let (mut min_toi, mut min_toi_entity) = (delta_secs, None);

        // Iterate through colliders intersecting the AABB of the CCD body.
        let intersecting_entities = contact_graph.entities_colliding_with(entity);
        for (collider2, &ColliderOf { body: entity2 }) in colliders.iter_many(intersecting_entities)
        {
            debug_assert_ne!(entity, entity2, "collider AABB cannot intersect itself");

            // Get the body associated with the collider.
            // Safety: `AabbIntersections` should never contain the entity of a collider
            //         attached to the first body, and the entities are also ensured to be different above.
            if let Ok(body2) = unsafe { bodies.get_unchecked(entity2) } {
                if !ccd1.include_dynamic && body2.rb.is_dynamic() {
                    continue;
                }

                // Get the solver body associated with the second body.
                let solver_body2 = body2
                    .solver_body
                    .map_or(&dummy_body, |body| body.into_inner());

                let prev_pos2 = *body2.pos;
                let prev_rot2 = *body2.rot;
                let lin_vel2 = solver_body2.linear_velocity;
                let ang_vel2 = solver_body2.angular_velocity;

                #[cfg(feature = "2d")]
                let ang_vel_below_threshold = (ang_vel1 - ang_vel2).abs() < ccd1.angular_threshold;
                #[cfg(feature = "3d")]
                let ang_vel_below_threshold =
                    (ang_vel1 - ang_vel2).length_squared() < ccd1.angular_threshold.powi(2);

                // If both the relative linear and relative angular velocity
                // are below the defined thresholds, skip this body.
                if ang_vel_below_threshold
                    && (lin_vel1 - lin_vel2).length_squared() < ccd1.linear_threshold.powi(2)
                {
                    continue;
                }

                let iso1 = make_pose(prev_pos1, prev_rot1);
                let iso2 = make_pose(prev_pos2, prev_rot2);

                // TODO: Support child colliders
                let motion1 = NonlinearRigidMotion::new(
                    iso1,
                    com1.0.into(),
                    lin_vel1.into(),
                    ang_vel1.into(),
                );
                let motion2 = NonlinearRigidMotion::new(
                    iso2,
                    body2.com.0.into(),
                    lin_vel2.into(),
                    ang_vel2.into(),
                );

                let sweep_mode = if ccd1.mode == SweepMode::Linear
                    && body2.ccd.is_none_or(|ccd| ccd.mode == SweepMode::Linear)
                {
                    SweepMode::Linear
                } else {
                    SweepMode::NonLinear
                };

                if let Some(toi) = compute_ccd_toi(
                    sweep_mode,
                    &motion1,
                    collider1,
                    &motion2,
                    collider2,
                    min_toi,
                    narrow_phase_config.default_speculative_margin,
                ) {
                    min_toi = toi;
                    min_toi_entity = Some(entity2);
                }
            }
        }

        // Advance the bodies from the previous poses to the first time of impact.
        if let Some(Ok(body2)) =
            min_toi_entity.map(|entity| unsafe { bodies.get_unchecked(entity) })
        {
            // Get the solver body for the entity that was hit.
            let solver_body2 = body2
                .solver_body
                .map_or(&mut dummy_body, |body| body.into_inner());

            let lin_vel2 = solver_body2.linear_velocity;
            let ang_vel2 = solver_body2.angular_velocity;

            // Overshoot slightly to make sure the bodies advance and don't get stuck.
            let min_toi = min_toi * 1.0001;

            solver_body1.delta_position = min_toi * lin_vel1;

            // TODO: Abstract the integration logic to reuse it here
            #[cfg(feature = "2d")]
            {
                solver_body1.delta_rotation = Rotation::radians(ang_vel1 * min_toi);
            }
            #[cfg(feature = "3d")]
            {
                let delta_rot = Quaternion::from_scaled_axis(ang_vel1 * min_toi);
                solver_body1.delta_rotation.0 = delta_rot * solver_body1.delta_rotation.0;
            }

            solver_body2.delta_position = min_toi * lin_vel2;

            #[cfg(feature = "2d")]
            {
                solver_body2.delta_rotation = Rotation::radians(ang_vel2 * min_toi);
            }
            #[cfg(feature = "3d")]
            {
                let delta_rot = Quaternion::from_scaled_axis(ang_vel2 * min_toi);
                solver_body2.delta_rotation.0 = delta_rot * solver_body2.delta_rotation.0;
            }
        }
    }

    diagnostics.swept_ccd += start.elapsed();
}

/// Computes the time of impact for the motion of two objects for Continuous Collision Detection.
/// If the TOI is larger than `min_toi` or the shapes never touch, `None` is returned.
#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
fn compute_ccd_toi(
    mode: SweepMode,
    motion1: &NonlinearRigidMotion,
    collider1: &Collider,
    motion2: &NonlinearRigidMotion,
    collider2: &Collider,
    min_toi: Scalar,
    prediction_distance: Scalar,
) -> Option<Scalar> {
    if mode == SweepMode::Linear {
        let hit = cast_shapes(
            &motion1.start,
            motion1.linvel,
            collider1.shape_scaled().as_ref(),
            &motion2.start,
            motion2.linvel,
            collider2.shape_scaled().as_ref(),
            ShapeCastOptions {
                max_time_of_impact: min_toi,
                stop_at_penetration: false,
                ..default()
            },
        )
        .ok()??;
        let toi = hit.time_of_impact;
        if toi > 0.0 && toi < min_toi {
            // New smaller time of impact found
            return Some(toi);
        } else if toi == 0.0 {
            // If the time of impact is zero, fall back to computing the TOI of a small circle
            // around the centroid of the first body.
            let hit = cast_shapes(
                &motion1.start,
                motion1.linvel,
                collider1.shape_scaled().as_ref(),
                &motion2.start,
                motion2.linvel,
                &parry::shape::Ball::new(prediction_distance),
                ShapeCastOptions {
                    max_time_of_impact: min_toi,
                    stop_at_penetration: false,
                    ..default()
                },
            )
            .ok()??;
            let toi = hit.time_of_impact;
            if toi > 0.0 && toi < min_toi {
                // New smaller time of impact found
                return Some(toi);
            }
        }
    } else if let Ok(Some(ShapeCastHit {
        time_of_impact: toi,
        ..
    })) = cast_shapes_nonlinear(
        motion1,
        collider1.shape_scaled().as_ref(),
        motion2,
        collider2.shape_scaled().as_ref(),
        0.0,
        min_toi,
        false,
    ) {
        if toi > 0.0 && toi < min_toi {
            // New smaller time of impact found
            return Some(toi);
        } else if toi == 0.0 {
            // If the time of impact is zero, fall back to computing the TOI of a small circle
            // around the centroid of the first body.
            let hit = cast_shapes_nonlinear(
                motion1,
                collider1.shape_scaled().as_ref(),
                motion2,
                &parry::shape::Ball::new(prediction_distance),
                0.0,
                min_toi,
                false,
            )
            .ok()??;
            let toi = hit.time_of_impact;
            if toi > 0.0 && toi < min_toi {
                // New smaller time of impact found
                return Some(toi);
            }
        }
    }

    None
}
