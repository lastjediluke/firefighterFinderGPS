const functions = require('firebase-functions');
const admin = require('firebase-admin');
var GPS = require('gps');
admin.initializeApp();

// // Create and Deploy Your First Cloud Functions
// // https://firebase.google.com/docs/functions/write-firebase-functions
//
// exports.helloWorld = functions.https.onRequest((request, response) => {
//  response.send("Hello from Firebase!");
// });

exports.helloPubSub = functions.pubsub.topic('events').onPublish((message) => {

    var mGps = new GPS;

    // grabs the parsed object
    var getParsed;
    
    // turns message into base64...this one works
    const messageBody = message.data ? Buffer.from(message.data, 'base64').toString() : null;

    // find the gps entry, and cut it from the message
    var indBeg = messageBody.indexOf('gps');
    // console.log(indBeg);
    var indEnd = messageBody.indexOf('}');
    // console.log(indEnd);

    // get the "gps" till right before the end
    var gpsData = messageBody.substring(indBeg - 1, indEnd - 1);
    console.log("GPS: " + gpsData);

    // get the gngga sentence from the message
    var nggaBeg = gpsData.indexOf('$GNGG');
    var nggaEnd = gpsData.indexOf('\n', nggaBeg + 1);
    var gngga = gpsData.substring(nggaBeg, nggaEnd);
    // console.log("GNGGA not found");


    
    /*
    var a = nggaBeg;
    var b = 0;
    var gngga = '\0';
    while(gpsData[a] != '\n')
    {
        gngga[b] = gpsData[a];
        a = a + 1;
        b = b + 1;
    }
    */
    

    // these might need to change depending if we get NGGA or GNGGA data
    // var gngga = gpsData.slice(nggaBeg, nggaEnd - 1);
    console.log("GNGGA: " + gngga);

    // if we find the bad $NGGA data, which we normally do, then we fix it
    // if (gngga.indexOf("GNGGA") === -1)
    // {
      //   console.log
        // gngga = gngga.replace("$NGGA", "$GNGGA");
    // }

    // the parser stuff
    mGps.on('data', function(parsed) {
        getParsed = parsed;
        console.log(parsed);
    });
    mGps.update(gngga);
    console.log("Get Parsed: " + getParsed);
    console.log("Lat: " + getParsed.lat);
    console.log("Long: " + getParsed.lon);

    // removes comma, but not closing bracket
    var newMsg = messageBody.slice(0, indBeg - 2);
    newMsg = newMsg + '}';
    console.log("New: " + newMsg);

    // take the msg and turn it into json
    var MsgToJson = JSON.parse(newMsg);
    console.log(MsgToJson.name);
    console.log(MsgToJson.temp);
    console.log(MsgToJson.floor);
    console.log(MsgToJson.status);

    // var floorInt = parseInt(MsgToJson.floor, 10);
    
    console.log( "Old: " + messageBody);

    // I don't think the try block is working
    // try 
    // {
    // } catch (e) {
    //     console.error('PubSub message was not JSON', e);
    // }

    // this return updates the db
    var lat = 0;
    var long = 0;

    if (getParsed.lat === null && getParsed.lon === null)
    {
        long = -121.88024;
        lat = 37.337032;
        console.log("Null data");
    }
    else 
    {
        long = getParsed.lon;
        lat = getParsed.lat;
    }
    return admin.database().ref("/Active/" + MsgToJson.name).update({
        lat: lat,
        long: long,
        temp: MsgToJson.temp,
        floor: MsgToJson.floor,
        status: MsgToJson.status
    });
  });
