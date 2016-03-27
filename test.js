import sprintf from 'sprintf';

const opts = {
    set: 200,
    hysteresis: 30,
}

function control(state, { pressure }) {
    const { call, set, hysteresis } = state;
    const lowMark = set - hysteresis / 2;
    const highMark = set + hysteresis / 2;
    let newCall = call;
    if (call === 'off') {
        if (pressure < lowMark)
            newCall = 'high';
    } else if (call === 'high') {
        if (pressure > highMark)
            newCall = 'off';
        else if (pressure > highMark - hysteresis / 3)
            newCall = 'low';
    } else if (call === 'low') {
        if (pressure > highMark)
            newCall = 'off';
        else if (pressure < lowMark + hysteresis / 3)
            newCall = 'high';
    }
    return { ...state, call: newCall };
}

const [ Kp, Ki, Kd ] = [ 0.1, 0.001, 0.1 ];

function controlPid(state, { pressure }) {
    let { call, set, hysteresis, integral, lastDelta } = state;

    const lowMark = set - hysteresis / 2;
    const highMark = set + hysteresis / 2;

    if (call === 0) {
        if (pressure > lowMark)
            return state;
        call = 1;
    }

    if (pressure > highMark) {
        return { ...state, call: 0 };
    }

    const delta = set - pressure;
    let p = Kp * delta;
    let i = integral + Ki * delta;
    let d = Kd * (delta - lastDelta);
    let newCall = p + i + d;
    console.log(sprintf('P %6.2f I %6.2f D %6.2f    Call %6.2f', p, i, d, newCall));
    if (newCall > 1.0)
        newCall = 1.0;
    else if (newCall < 0.4)
        newCall = 0.4;
    else
        integral += Ki * delta;

    newCall = call * 0.8 + newCall * 0.2;

    return { ...state, call: newCall, integral, lastDelta: delta };
}

let state = { call: 1.0, set: 230, hysteresis: 50, integral: 0, lastDelta: 0 };
let pressure = 215;

setInterval(() => {
    state = controlPid(state, { pressure: pressure + Math.random() * 5 - 2.5 });
    pressure -= 0.4;
    pressure += 0.5 * state.call;
    console.log(state.call, pressure);
}, 100);
