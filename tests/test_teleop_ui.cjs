// Unit-test the page script against a fake DOM/transport. No browser or robot I/O.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(require('node:path').join(__dirname,'../tools/teleop.html'),'utf8');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1].replace('__SESSION_TOKEN__',JSON.stringify('test'));
const events={},nodes={},requests=[];
let state={mode:'disarmed',generation:0,message:'test',speed:20,left:0,right:0,fault:0};
const document={hidden:false,hasFocus:()=>true,getElementById:id=>nodes[id]??=( {textContent:'',disabled:false}),querySelectorAll:()=>[],addEventListener:(key,fn)=>events[key]=fn};
const context=vm.createContext({document,window:{addEventListener:(key,fn)=>events[key]=fn},crypto:{randomUUID:()=> 'test'},setInterval:()=>{},fetch:async(url,options)=>{if(options){const body=JSON.parse(options.body);requests.push(body);if(body.action==='stop')state={...state,mode:'disarmed',generation:state.generation+1};}return {ok:true,json:async()=>({...state})};}});
vm.runInContext(code,context);
const evalCode=s=>vm.runInContext(s,context);
const key=(code,extra={})=>({code,preventDefault(){},repeat:false,metaKey:false,ctrlKey:false,altKey:false,...extra});
const vector=()=>JSON.parse(evalCode('JSON.stringify(vector())'));
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function arm(){await flush();state={...state,mode:'armed',generation:state.generation+1};context.fixture=state;evalCode('render(fixture); active=true;');}
(async()=>{
 await arm(); events.keydown(key('KeyW'));assert.deepEqual(vector(),[1,1]);
 events.keydown(key('KeyA'));assert.deepEqual(vector(),[0,1]);
 events.keyup(key('KeyW'));assert.deepEqual(vector(),[-1,1]);
 events.keyup(key('KeyA'));assert.deepEqual(vector(),[0,0]);await flush();
 events.keydown(key('ArrowUp'));events.keydown(key('ArrowDown'));assert.deepEqual(vector(),[0,0]);
 events.keydown(key('Space'));await flush();assert.equal(requests.at(-1).action,'stop');assert.equal(evalCode('active'),false);
 await arm();events.keydown(key('KeyW',{repeat:true}));assert.deepEqual(vector(),[0,0]);
 events.keydown(key('KeyW',{metaKey:true}));assert.deepEqual(vector(),[0,0]);
 events.keydown(key('KeyD'));assert.deepEqual(vector(),[1,-1]);events.blur();await flush();assert.equal(requests.at(-1).action,'stop');assert.deepEqual(vector(),[0,0]);
 await arm();events.keydown(key('KeyW'));document.hidden=true;events.visibilitychange();await flush();assert.equal(requests.at(-1).action,'stop');document.hidden=false;
 await arm();events.keydown(key('KeyW'));context.fixture={...state,generation:state.generation+1};evalCode('render(fixture)');assert.deepEqual(vector(),[0,0]);
 context.fixture={...state,mode:'armed',generation:0};const gen=evalCode('generation');evalCode('render(fixture)');assert.equal(evalCode('generation'),gen);
 console.log('PASS: WASD/arrows, key combinations/release, Space, focus/visibility loss, repeat/modifier filtering, fresh-key arming and stale response guards');
})().catch(error=>{console.error(error);process.exitCode=1;});
